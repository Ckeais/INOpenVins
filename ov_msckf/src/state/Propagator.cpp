/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2022 Patrick Geneva
 * Copyright (C) 2022 Guoquan Huang
 * Copyright (C) 2022 OpenVINS Contributors
 * Copyright (C) 2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "Propagator.h"
#include <unsupported/Eigen/MatrixFunctions>




using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;
using namespace Eigen;


void Propagator::propagate_and_clone(std::shared_ptr<State> state, double timestamp) {

  // If the difference between the current update time and state is zero
  // We should crash, as this means we would have two clones at the same time!!!!
  if (state->_timestamp == timestamp) {
    PRINT_ERROR(RED "Propagator::propagate_and_clone(): Propagation called again at same timestep at last update timestep!!!!\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // We should crash if we are trying to propagate backwards
  if (state->_timestamp > timestamp) {
    PRINT_ERROR(RED "Propagator::propagate_and_clone(): Propagation called trying to propagate backwards in time!!!!\n" RESET);
    PRINT_ERROR(RED "Propagator::propagate_and_clone(): desired propagation = %.4f\n" RESET, (timestamp - state->_timestamp));
    std::exit(EXIT_FAILURE);
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Set the last time offset value if we have just started the system up
  if (!have_last_prop_time_offset) {
    last_prop_time_offset = state->_calib_dt_CAMtoIMU->value()(0);
    have_last_prop_time_offset = true;
  }

  // Get what our IMU-camera offset should be (t_imu = t_cam + calib_dt)
  double t_off_new = state->_calib_dt_CAMtoIMU->value()(0);

  // First lets construct an IMU vector of measurements we need
  double time0 = state->_timestamp + last_prop_time_offset;
  double time1 = timestamp + t_off_new;
  std::vector<ov_core::ImuData> prop_data;
  {
    std::lock_guard<std::mutex> lck(imu_data_mtx);
    prop_data = Propagator::select_imu_readings(imu_data, time0, time1);
  }

  // We are going to sum up all the state transition matrices, so we can do a single large multiplication at the end
  // Phi_summed = Phi_i*Phi_summed
  // Q_summed = Phi_i*Q_summed*Phi_i^T + Q_i
  // After summing we can multiple the total phi to get the updated covariance
  // We will then add the noise to the IMU portion of the state
  Eigen::Matrix<double, 15, 15> Phi_summed = Eigen::Matrix<double, 15, 15>::Identity();
  Eigen::Matrix<double, 15, 15> Qd_summed = Eigen::Matrix<double, 15, 15>::Zero();
  double dt_summed = 0;

  // Loop through all IMU messages, and use them to move the state forward in time
  // This uses the zero'th order quat, and then constant acceleration discrete
  if (prop_data.size() > 1) {
    for (size_t i = 0; i < prop_data.size() - 1; i++) {

      // Get the next state Jacobian and noise Jacobian for this IMU reading
      Eigen::Matrix<double, 15, 15> PHI = Eigen::Matrix<double, 15, 15>::Zero();
      Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();
      Eigen::Matrix<double, 15, 15> Qdi = Eigen::Matrix<double, 15, 15>::Zero();
      Eigen::Matrix<double, 15, 1> mu = Eigen::Matrix<double, 15, 1>::Zero();
      predict_and_compute(state, prop_data.at(i), prop_data.at(i + 1), F, PHI, Qdi, mu);

      // Next we should propagate our IMU covariance
      // Pii' = F*Pii*F.transpose() + G*Q*G.transpose()
      // Pci' = F*Pci and Pic' = Pic*F.transpose()
      // NOTE: Here we are summing the state transition F so we can do a single mutiplication later
      // NOTE: Phi_summed = Phi_i*Phi_summed
      // NOTE: Q_summed = Phi_i*Q_summed*Phi_i^T + G*Q_i*G^T
      Phi_summed = F * Phi_summed;
      Qd_summed = PHI * Qd_summed * PHI.transpose() + Qdi;
      Qd_summed = 0.5 * (Qd_summed + Qd_summed.transpose());
      dt_summed += prop_data.at(i + 1).timestamp - prop_data.at(i).timestamp;

      // std::cout << "calculated Qd_summed\n";




      // unscented transform from se4(3) to original state
      int n = 15;
      int k = 2;

      Eigen::LLT<MatrixXd> lltOfQd_summed(Qd_summed);
      Eigen::MatrixXd L = lltOfQd_summed.matrixL();
      Eigen::MatrixXd L_prime = sqrt(n + k)*L;

      Eigen::Matrix<double, 15, 1> sigma_points [31];
      double weights [31];

      sigma_points[0] = mu;
      weights[0] = (1.0*k)/(n + k);

      int j = 1;
      while (j < 16) {
        sigma_points[j] = mu + L_prime.col(j-1);
        weights[j] = 1.0/(2*(n + k));
        j++;
      }

      j = 16;
      while (j < 31) {
        sigma_points[j] = mu + L_prime.col(j-16);
        weights[j] = 1.0/(2*(n + k));
        j++;
      }


      Eigen::Matrix<double, 15, 1> transformed_points [31];
      // std::cout << "test1\n";

      for (int i = 0; i < 31; i++) {
        Eigen::Matrix<double, 7, 7> mu_hat = Eigen::Matrix<double, 7, 7>::Zero();
        mu_hat(0, 1) = -sigma_points[i](2);
        mu_hat(0, 2) = sigma_points[i](1);
        mu_hat(1, 0) = sigma_points[i](2);
        mu_hat(1, 2) = -sigma_points[i](0);
        mu_hat(2, 0) = -sigma_points[i](1);
        mu_hat(2, 1) = sigma_points[i](0);
        // std::cout << "test2\n";
        mu_hat.block(0, 3, 3, 1) = sigma_points[i].block(3, 0, 3, 1);
        mu_hat.block(0, 4, 3, 1) = sigma_points[i].block(6, 0, 3, 1);
        mu_hat.block(0, 5, 3, 1) = sigma_points[i].block(9, 0, 3, 1);
        mu_hat.block(0, 6, 3, 1) = sigma_points[i].block(12, 0, 3, 1);
        // std::cout << "test3\n";
        Eigen::Matrix<double, 7, 7> SE3_point = Eigen::Matrix<double, 7, 7>::Zero();
        SE3_point.block(0, 0, 3, 3) = (mu_hat.block(0, 0, 3, 3)).exp();
        double theta = (sigma_points[i].segment(0, 3)).norm();
        Eigen::Matrix<double, 3, 3> J = Eigen::Matrix<double, 3, 3>::Identity() + ((1 - cos(theta))/(theta*theta))*mu_hat.block(0, 0, 3, 3) + ((theta - sin(theta))/(theta*theta*theta))*(mu_hat.block(0, 0, 3, 3)*mu_hat.block(0, 0, 3, 3));
        SE3_point.block(0, 3, 3, 1) = J*sigma_points[i].block(3, 0, 3, 1);
        SE3_point.block(0, 4, 3, 1) = J*sigma_points[i].block(6, 0, 3, 1);
        SE3_point.block(0, 5, 3, 1) = J*sigma_points[i].block(9, 0, 3, 1);
        SE3_point.block(0, 6, 3, 1) = J*sigma_points[i].block(12, 0, 3, 1);
        SE3_point(3, 3) = 1;
        SE3_point(4, 4) = 1;
        SE3_point(5, 5) = 1;
        SE3_point(6, 6) = 1;

        Eigen::Matrix<double, 3, 1> w_temp = Eigen::Matrix<double, 3, 1>::Zero();
        w_temp(0) = sigma_points[i](0);
        w_temp(1) = sigma_points[i](1);
        w_temp(2) = sigma_points[i](2);
        transformed_points[i].block(0, 0, 3, 1) = w_temp;
        transformed_points[i].block(3, 0, 3, 1) = SE3_point.block(0, 3, 3, 1);
        transformed_points[i].block(6, 0, 3, 1) = SE3_point.block(0, 4, 3, 1);
        transformed_points[i].block(9, 0, 3, 1) = SE3_point.block(0, 5, 3, 1);
        transformed_points[i].block(12, 0, 3, 1) = SE3_point.block(0, 6, 3, 1);
        // std::cout << "test4\n";

      }

      Eigen::Matrix<double, 15, 1> mu_prime = Eigen::Matrix<double, 15, 1>::Zero();
      Eigen::Matrix<double, 15, 15> Qd_summed_prime = Eigen::Matrix<double, 15, 15>::Zero();

      for (int i = 0; i < 31; i++) {
        mu_prime = mu_prime + weights[i]*transformed_points[i];
      }

      for (int i = 0; i < 31; i++) {
        Qd_summed_prime = Qd_summed_prime + weights[i]*((transformed_points[i] - mu_prime)*(transformed_points[i] - mu_prime).transpose());
      }

      Qd_summed = Qd_summed_prime;


    }
  }

  // Last angular velocity (used for cloning when estimating time offset)
  Eigen::Matrix<double, 3, 1> last_w = Eigen::Matrix<double, 3, 1>::Zero();
  if (prop_data.size() > 1)
    last_w = prop_data.at(prop_data.size() - 2).wm - state->_imu->bias_g();
  else if (!prop_data.empty())
    last_w = prop_data.at(prop_data.size() - 1).wm - state->_imu->bias_g();

  // Do the update to the covariance with our "summed" state transition and IMU noise addition...
  std::vector<std::shared_ptr<Type>> Phi_order;
  Phi_order.push_back(state->_imu);
  StateHelper::EKFPropagation(state, Phi_order, Phi_order, Phi_summed, Qd_summed);

  // Set timestamp data
  state->_timestamp = timestamp;
  last_prop_time_offset = t_off_new;

  // Now perform stochastic cloning
  StateHelper::augment_clone(state, last_w);
}

bool Propagator::fast_state_propagate(std::shared_ptr<State> state, double timestamp, Eigen::Matrix<double, 13, 1> &state_plus,
                                      Eigen::Matrix<double, 12, 12> &covariance) {

  // First we will store the current calibration / estimates of the state
  double state_time = state->_timestamp;
  Eigen::MatrixXd state_est = state->_imu->value();
  Eigen::MatrixXd state_covariance = StateHelper::get_marginal_covariance(state, {state->_imu});
  double t_off = state->_calib_dt_CAMtoIMU->value()(0);

  // First lets construct an IMU vector of measurements we need
  double time0 = state_time + t_off;
  double time1 = timestamp + t_off;
  std::vector<ov_core::ImuData> prop_data;
  {
    std::lock_guard<std::mutex> lck(imu_data_mtx);
    prop_data = Propagator::select_imu_readings(imu_data, time0, time1, false);
  }
  if (prop_data.size() < 2)
    return false;

  // Biases
  Eigen::Vector3d bias_g = state_est.block(10, 0, 3, 1);
  Eigen::Vector3d bias_a = state_est.block(13, 0, 3, 1);

  // Loop through all IMU messages, and use them to move the state forward in time
  // This uses the zero'th order quat, and then constant acceleration discrete
  for (size_t i = 0; i < prop_data.size() - 1; i++) {

    // Corrected imu measurements
    double dt = prop_data.at(i + 1).timestamp - prop_data.at(i).timestamp;
    Eigen::Vector3d w_hat = 0.5 * (prop_data.at(i + 1).wm + prop_data.at(i).wm) - bias_g;
    Eigen::Vector3d a_hat = 0.5 * (prop_data.at(i + 1).am + prop_data.at(i).am) - bias_a;
    Eigen::Matrix3d R_Gtoi = quat_2_Rot(state_est.block(0, 0, 4, 1));
    Eigen::Vector3d v_iinG = state_est.block(7, 0, 3, 1);
    Eigen::Vector3d p_iinG = state_est.block(4, 0, 3, 1);

    // State transition and noise matrix
    Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();
    Eigen::Matrix<double, 15, 15> Qd = Eigen::Matrix<double, 15, 15>::Zero();
    F.block(0, 0, 3, 3) = exp_so3(-w_hat * dt);
    F.block(0, 9, 3, 3).noalias() = -exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
    F.block(9, 9, 3, 3).setIdentity();
    F.block(6, 0, 3, 3).noalias() = -R_Gtoi.transpose() * skew_x(a_hat * dt);
    F.block(6, 6, 3, 3).setIdentity();
    F.block(6, 12, 3, 3) = -R_Gtoi.transpose() * dt;
    F.block(12, 12, 3, 3).setIdentity();
    F.block(3, 0, 3, 3).noalias() = -0.5 * R_Gtoi.transpose() * skew_x(a_hat * dt * dt);
    F.block(3, 6, 3, 3) = Eigen::Matrix3d::Identity() * dt;
    F.block(3, 12, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
    F.block(3, 3, 3, 3).setIdentity();
    Eigen::Matrix<double, 15, 12> G = Eigen::Matrix<double, 15, 12>::Zero();
    G.block(0, 0, 3, 3) = -exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
    G.block(6, 3, 3, 3) = -R_Gtoi.transpose() * dt;
    G.block(3, 3, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
    G.block(9, 6, 3, 3).setIdentity();
    G.block(12, 9, 3, 3).setIdentity();

    // Construct our discrete noise covariance matrix
    // Note that we need to convert our continuous time noises to discrete
    // Equations (129) amd (130) of Trawny tech report
    Eigen::Matrix<double, 12, 12> Qc = Eigen::Matrix<double, 12, 12>::Zero();
    Qc.block(0, 0, 3, 3) = _noises.sigma_w_2 / dt * Eigen::Matrix3d::Identity();
    Qc.block(3, 3, 3, 3) = _noises.sigma_a_2 / dt * Eigen::Matrix3d::Identity();
    Qc.block(6, 6, 3, 3) = _noises.sigma_wb_2 * dt * Eigen::Matrix3d::Identity();
    Qc.block(9, 9, 3, 3) = _noises.sigma_ab_2 * dt * Eigen::Matrix3d::Identity();
    Qd = G * Qc * G.transpose();
    Qd = 0.5 * (Qd + Qd.transpose());
    state_covariance = F * state_covariance * F.transpose() + Qd;

    // Propagate the mean forward
    state_est.block(0, 0, 4, 1) = rot_2_quat(exp_so3(-w_hat * dt) * R_Gtoi);
    state_est.block(4, 0, 3, 1) = p_iinG + v_iinG * dt + 0.5 * R_Gtoi.transpose() * a_hat * dt * dt - 0.5 * _gravity * dt * dt;
    state_est.block(7, 0, 3, 1) = v_iinG + R_Gtoi.transpose() * a_hat * dt - _gravity * dt;
  }

  // Now record what the predicted state should be
  Eigen::Vector4d q_Gtoi = state_est.block(0, 0, 4, 1);
  Eigen::Vector3d v_iinG = state_est.block(7, 0, 3, 1);
  Eigen::Vector3d p_iinG = state_est.block(4, 0, 3, 1);
  state_plus.setZero();
  state_plus.block(0, 0, 4, 1) = q_Gtoi;
  state_plus.block(4, 0, 3, 1) = p_iinG;
  state_plus.block(7, 0, 3, 1) = quat_2_Rot(q_Gtoi) * v_iinG;
  state_plus.block(10, 0, 3, 1) = 0.5 * (prop_data.at(prop_data.size() - 1).wm + prop_data.at(prop_data.size() - 2).wm) - bias_g;

  // Do a covariance propagation for our velocity
  // TODO: more properly do the covariance of the angular velocity here...
  // TODO: it should be dependent on the state bias, thus correlated with the pose
  covariance.setZero();
  Eigen::Matrix<double, 15, 15> Phi = Eigen::Matrix<double, 15, 15>::Identity();
  Phi.block(6, 6, 3, 3) = quat_2_Rot(q_Gtoi);
  state_covariance = Phi * state_covariance * Phi.transpose();
  covariance.block(0, 0, 9, 9) = state_covariance.block(0, 0, 9, 9);
  double dt = prop_data.at(prop_data.size() - 1).timestamp + prop_data.at(prop_data.size() - 2).timestamp;
  covariance.block(9, 9, 3, 3) = _noises.sigma_w_2 / dt * Eigen::Matrix3d::Identity();
  return true;
}

std::vector<ov_core::ImuData> Propagator::select_imu_readings(const std::vector<ov_core::ImuData> &imu_data, double time0, double time1,
                                                              bool warn) {

  // Our vector imu readings
  std::vector<ov_core::ImuData> prop_data;

  // Ensure we have some measurements in the first place!
  if (imu_data.empty()) {
    if (warn)
      PRINT_WARNING(YELLOW "Propagator::select_imu_readings(): No IMU measurements. IMU-CAMERA are likely messed up!!!\n" RESET);
    return prop_data;
  }

  // Loop through and find all the needed measurements to propagate with
  // Note we split measurements based on the given state time, and the update timestamp
  for (size_t i = 0; i < imu_data.size() - 1; i++) {

    // START OF THE INTEGRATION PERIOD
    // If the next timestamp is greater then our current state time
    // And the current is not greater then it yet...
    // Then we should "split" our current IMU measurement
    if (imu_data.at(i + 1).timestamp > time0 && imu_data.at(i).timestamp < time0) {
      ov_core::ImuData data = Propagator::interpolate_data(imu_data.at(i), imu_data.at(i + 1), time0);
      prop_data.push_back(data);
      // PRINT_DEBUG("propagation #%d = CASE 1 = %.3f => %.3f\n",
      // (int)i,data.timestamp-prop_data.at(0).timestamp,time0-prop_data.at(0).timestamp);
      continue;
    }

    // MIDDLE OF INTEGRATION PERIOD
    // If our imu measurement is right in the middle of our propagation period
    // Then we should just append the whole measurement time to our propagation vector
    if (imu_data.at(i).timestamp >= time0 && imu_data.at(i + 1).timestamp <= time1) {
      prop_data.push_back(imu_data.at(i));
      // PRINT_DEBUG("propagation #%d = CASE 2 = %.3f\n",(int)i,imu_data.at(i).timestamp-prop_data.at(0).timestamp);
      continue;
    }

    // END OF THE INTEGRATION PERIOD
    // If the current timestamp is greater then our update time
    // We should just "split" the NEXT IMU measurement to the update time,
    // NOTE: we add the current time, and then the time at the end of the interval (so we can get a dt)
    // NOTE: we also break out of this loop, as this is the last IMU measurement we need!
    if (imu_data.at(i + 1).timestamp > time1) {
      // If we have a very low frequency IMU then, we could have only recorded the first integration (i.e. case 1) and nothing else
      // In this case, both the current IMU measurement and the next is greater than the desired intepolation, thus we should just cut the
      // current at the desired time Else, we have hit CASE2 and this IMU measurement is not past the desired propagation time, thus add the
      // whole IMU reading
      if (imu_data.at(i).timestamp > time1 && i == 0) {
        // This case can happen if we don't have any imu data that has occured before the startup time
        // This means that either we have dropped IMU data, or we have not gotten enough.
        // In this case we can't propgate forward in time, so there is not that much we can do.
        break;
      } else if (imu_data.at(i).timestamp > time1) {
        ov_core::ImuData data = interpolate_data(imu_data.at(i - 1), imu_data.at(i), time1);
        prop_data.push_back(data);
        // PRINT_DEBUG("propagation #%d = CASE 3.1 = %.3f => %.3f\n",
        // (int)i,imu_data.at(i).timestamp-prop_data.at(0).timestamp,imu_data.at(i).timestamp-time0);
      } else {
        prop_data.push_back(imu_data.at(i));
        // PRINT_DEBUG("propagation #%d = CASE 3.2 = %.3f => %.3f\n",
        // (int)i,imu_data.at(i).timestamp-prop_data.at(0).timestamp,imu_data.at(i).timestamp-time0);
      }
      // If the added IMU message doesn't end exactly at the camera time
      // Then we need to add another one that is right at the ending time
      if (prop_data.at(prop_data.size() - 1).timestamp != time1) {
        ov_core::ImuData data = interpolate_data(imu_data.at(i), imu_data.at(i + 1), time1);
        prop_data.push_back(data);
        // PRINT_DEBUG("propagation #%d = CASE 3.3 = %.3f => %.3f\n", (int)i,data.timestamp-prop_data.at(0).timestamp,data.timestamp-time0);
      }
      break;
    }
  }

  // Check that we have at least one measurement to propagate with
  if (prop_data.empty()) {
    if (warn)
      PRINT_WARNING(
          YELLOW
          "Propagator::select_imu_readings(): No IMU measurements to propagate with (%d of 2). IMU-CAMERA are likely messed up!!!\n" RESET,
          (int)prop_data.size());
    return prop_data;
  }

  // If we did not reach the whole integration period (i.e., the last inertial measurement we have is smaller then the time we want to
  // reach) Then we should just "stretch" the last measurement to be the whole period (case 3 in the above loop)
  // if(time1-imu_data.at(imu_data.size()-1).timestamp > 1e-3) {
  //    PRINT_DEBUG(YELLOW "Propagator::select_imu_readings(): Missing inertial measurements to propagate with (%.6f sec missing).
  //    IMU-CAMERA are likely messed up!!!\n" RESET, (time1-imu_data.at(imu_data.size()-1).timestamp)); return prop_data;
  //}

  // Loop through and ensure we do not have an zero dt values
  // This would cause the noise covariance to be Infinity
  for (size_t i = 0; i < prop_data.size() - 1; i++) {
    if (std::abs(prop_data.at(i + 1).timestamp - prop_data.at(i).timestamp) < 1e-12) {
      if (warn)
        PRINT_WARNING(YELLOW "Propagator::select_imu_readings(): Zero DT between IMU reading %d and %d, removing it!\n" RESET, (int)i,
                      (int)(i + 1));
      prop_data.erase(prop_data.begin() + i);
      i--;
    }
  }

  // Check that we have at least one measurement to propagate with
  if (prop_data.size() < 2) {
    if (warn)
      PRINT_WARNING(
          YELLOW
          "Propagator::select_imu_readings(): No IMU measurements to propagate with (%d of 2). IMU-CAMERA are likely messed up!!!\n" RESET,
          (int)prop_data.size());
    return prop_data;
  }

  // Success :D
  return prop_data;
}

void Propagator::predict_and_compute(std::shared_ptr<State> state, const ov_core::ImuData &data_minus, const ov_core::ImuData &data_plus,
                                     Eigen::Matrix<double, 15, 15> &F, Eigen::Matrix<double, 15, 15> &PHI, Eigen::Matrix<double, 15, 15> &Qd, 
                                     Eigen::Matrix<double, 15, 1> &new_mu) {

  // std::cout << "running predict_and_computer\n";
  // Set them to zero
  F.setZero();
  Qd.setZero();


  // Time elapsed over interval
  double dt = data_plus.timestamp - data_minus.timestamp;
  // assert(data_plus.timestamp>data_minus.timestamp);

  // Corrected imu measurements
  Eigen::Matrix<double, 3, 1> w_hat = data_minus.wm - state->_imu->bias_g();
  Eigen::Matrix<double, 3, 1> a_hat = data_minus.am - state->_imu->bias_a();
  Eigen::Matrix<double, 3, 1> w_hat2 = data_plus.wm - state->_imu->bias_g();
  Eigen::Matrix<double, 3, 1> a_hat2 = data_plus.am - state->_imu->bias_a();

  // Compute the new state mean value
  Eigen::Vector4d new_q;
  Eigen::Vector3d new_v, new_p;
  if (state->_options.use_rk4_integration)
    predict_mean_rk4(state, dt, w_hat, a_hat, w_hat2, a_hat2, new_q, new_v, new_p);
  else
    predict_mean_discrete(state, dt, w_hat, a_hat, w_hat2, a_hat2, new_q, new_v, new_p);

  // Get the locations of each entry of the imu state
  int th_id = state->_imu->q()->id() - state->_imu->id();
  int p_id = state->_imu->p()->id() - state->_imu->id();
  int v_id = state->_imu->v()->id() - state->_imu->id();
  int bg_id = state->_imu->bg()->id() - state->_imu->id();
  int ba_id = state->_imu->ba()->id() - state->_imu->id();

  // Allocate noise Jacobian
  Eigen::Matrix<double, 15, 12> G = Eigen::Matrix<double, 15, 12>::Zero();
  // Eigen::Matrix<double, 5, 5> X_prev = Eigen::Matrix<double, 5, 5>::Zero();
  // Eigen::Matrix<double, 5, 5> X = Eigen::Matrix<double, 5, 5>::Zero();
  // MatrixXd A = Eigen::Matrix<double, 9, 9>::Zero();
  // MatrixXd PHI;
  // Eigen::Matrix<double, 9, 9> P = Eigen::Matrix<double, 9, 9>::Zero();
  // Eigen::Matrix<double, 9, 9> P_prev = Eigen::Matrix<double, 9, 9>::Zero();

  // Now compute Jacobian of new state wrt old state and noise
  if (state->_options.do_fej) {

    // This is the change in the orientation from the end of the last prop to the current prop
    // This is needed since we need to include the "k-th" updated orientation information
    Eigen::Matrix<double, 3, 3> Rfej = state->_imu->Rot_fej();
    Eigen::Matrix<double, 3, 3> dR = quat_2_Rot(new_q) * Rfej.transpose();

    Eigen::Matrix<double, 3, 1> v_fej = state->_imu->vel_fej();
    Eigen::Matrix<double, 3, 1> p_fej = state->_imu->pos_fej();

    F.block(th_id, th_id, 3, 3) = dR;
    F.block(th_id, bg_id, 3, 3).noalias() = -dR * Jr_so3(-w_hat * dt) * dt;
    // F.block(th_id, bg_id, 3, 3).noalias() = -dR * Jr_so3(-log_so3(dR)) * dt;
    F.block(bg_id, bg_id, 3, 3).setIdentity();
    F.block(v_id, th_id, 3, 3).noalias() = -skew_x(new_v - v_fej + _gravity * dt) * Rfej.transpose();
    // F.block(v_id, th_id, 3, 3).noalias() = -Rfej.transpose() * skew_x(Rfej*(new_v-v_fej+_gravity*dt));
    F.block(v_id, v_id, 3, 3).setIdentity();
    F.block(v_id, ba_id, 3, 3) = -Rfej.transpose() * dt;
    F.block(ba_id, ba_id, 3, 3).setIdentity();
    F.block(p_id, th_id, 3, 3).noalias() = -skew_x(new_p - p_fej - v_fej * dt + 0.5 * _gravity * dt * dt) * Rfej.transpose();
    // F.block(p_id, th_id, 3, 3).noalias() = -0.5 * Rfej.transpose() * skew_x(2*Rfej*(new_p-p_fej-v_fej*dt+0.5*_gravity*dt*dt));
    F.block(p_id, v_id, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dt;
    F.block(p_id, ba_id, 3, 3) = -0.5 * Rfej.transpose() * dt * dt;
    F.block(p_id, p_id, 3, 3).setIdentity();

    G.block(th_id, 0, 3, 3) = -dR * Jr_so3(-w_hat * dt) * dt;
    // G.block(th_id, 0, 3, 3) = -dR * Jr_so3(-log_so3(dR)) * dt;
    G.block(v_id, 3, 3, 3) = -Rfej.transpose() * dt;
    G.block(p_id, 3, 3, 3) = -0.5 * Rfej.transpose() * dt * dt;
    G.block(bg_id, 6, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    G.block(ba_id, 9, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

  } else {

    Eigen::Matrix<double, 3, 3> R_Gtoi = state->_imu->Rot();

    F.block(th_id, th_id, 3, 3) = exp_so3(-w_hat * dt);
    F.block(th_id, bg_id, 3, 3).noalias() = -exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
    F.block(bg_id, bg_id, 3, 3).setIdentity();
    F.block(v_id, th_id, 3, 3).noalias() = -R_Gtoi.transpose() * skew_x(a_hat * dt);
    F.block(v_id, v_id, 3, 3).setIdentity();
    F.block(v_id, ba_id, 3, 3) = -R_Gtoi.transpose() * dt;
    F.block(ba_id, ba_id, 3, 3).setIdentity();
    F.block(p_id, th_id, 3, 3).noalias() = -0.5 * R_Gtoi.transpose() * skew_x(a_hat * dt * dt);
    F.block(p_id, v_id, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dt;
    F.block(p_id, ba_id, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
    F.block(p_id, p_id, 3, 3).setIdentity();

    G.block(th_id, 0, 3, 3) = -exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
    G.block(v_id, 3, 3, 3) = -R_Gtoi.transpose() * dt;
    G.block(p_id, 3, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
    G.block(bg_id, 6, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    G.block(ba_id, 9, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

    // X_prev.block(th_id, 0, 3, 3) = quat_2_Rot(state->_imu->q());
    // X_prev.block(v_id, 3, 3, 3) = state->_imu->v();
    // X_prev.block(p_id, 3, 3, 3) = state->_imu->p();
    // X_prev(3,3) = 1;
    // X_prev(4,4) = 1;

    // Eigen::Matrix<double, 3, 3> w_hat_mat {     
    //     {0, -w_hat(2), w_hat(1)},
    //     {w_hat(2), 0, -w_hat(0)},
    //     {-w_hat(1), w_hat(0), 0}    
    // };

    // Eigen::Matrix<double, 3, 3> a_hat_mat {     
    //     {0, -a_hat(2), a_hat(1)},
    //     {a_hat(2), 0, -a_hat(0)},
    //     {-a_hat(1), a_hat(0), 0}    
    // };

    // A.block(0, 0, 3, 3) = -w_hat_mat;
    // A.block(3, 0, 3, 3) = -a_hat_mat;
    // A.block(3, 3, 3, 3) = -w_hat_mat;
    // A.block(6, 6, 3, 3) = -w_hat_mat;
    // A.block(6, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

    // PHI = (A*dt).Eigen::hat();



  }

  // X.block(0, 0, 3, 3) = quat_2_Rot(state->_imu->q())*w_hat_mat;
  // X.block(0, 3, 3, 3) = quat_2_Rot(state->_imu->q())*a_hat_mat + _gravity;
  // X.block(0, 6, 3, 3) = state->_imu->v();



  // Construct our discrete noise covariance matrix
  // Note that we need to convert our continuous time noises to discrete
  // Equations (129) amd (130) of Trawny tech report
  Eigen::Matrix<double, 12, 12> Qc = Eigen::Matrix<double, 12, 12>::Zero();
  Qc.block(0, 0, 3, 3) = _noises.sigma_w_2 / dt * Eigen::Matrix<double, 3, 3>::Identity();
  Qc.block(3, 3, 3, 3) = _noises.sigma_a_2 / dt * Eigen::Matrix<double, 3, 3>::Identity();
  Qc.block(6, 6, 3, 3) = _noises.sigma_wb_2 * dt * Eigen::Matrix<double, 3, 3>::Identity();
  Qc.block(9, 9, 3, 3) = _noises.sigma_ab_2 * dt * Eigen::Matrix<double, 3, 3>::Identity();

  // Compute the noise injected into the state over the interval
  Qd = G * Qc * G.transpose();
  Qd = 0.5 * (Qd + Qd.transpose());

  // double qr = state->_imu->q()[0];
  // double qx = state->_imu->q()[1];
  // double qy = state->_imu->q()[2];
  // double qz = state->_imu->q()[3];

  // double yaw = arctan((2*(qr*qz + qz*qy))/(1 - 2*(qx*qx + qz*qz)));
  // double pitch = arcsin(2*(qr*qy - qx*qz);
  // double roll = arctan((2*(qr*qx + qy*qz))/(1 - 2*(qx*qx + qy*qy)));

  // Eigen::Matrix<double, 6, 7> quat2EulJ = Eigen::Matrix<double, 6, 7>::Zero();

  // quat2EulJ.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

  // Eigen::Matrix<double, 3, 4> quat2EulubJ = Eigen::Matrix<double, 3, 4>::Zero();

  // quat2EulubJ(0, 0) = (2*qz)/((1 - 2*(qz*qz + qx*qx))*((4*(qz*qr + qy*qz)*(qz*qr + qy*qz))/(1 - 2*(qz*qz + qx*qx)*(qz*qz + qx*qx))) + 1)
  // quat2EulubJ(0, 1) = 
  // quat2EulubJ(0, 2) = 
  // quat2EulubJ(0, 3) = 

  // quat2EulubJ(1, 0) = 
  // quat2EulubJ(1, 1) = 
  // quat2EulubJ(1, 2) = 
  // quat2EulubJ(1, 3) = 

  // quat2EulubJ(2, 0) = 
  // quat2EulubJ(2, 1) = 
  // quat2EulubJ(2, 2) = 
  // quat2EulubJ(2, 3) = 


  // quat2EulJ.block(3, 3, 3, 4) = quat2EulubJ;

  // unscented transform from original state to se4(3)

  int n = 15;
  int k = 2;

  Eigen::LLT<MatrixXd> lltOfQd(Qd);
  Eigen::MatrixXd L = lltOfQd.matrixL();
  Eigen::MatrixXd L_prime = sqrt(n + k)*L;

  Eigen::Matrix<double, 15, 1> sigma_points [31];
  double weights [31];

  // VectorXd vec_joined(state->_imu->q().size() + state->_imu->q().size());
  // vec_joined << vec1, vec2;
  Eigen::Matrix<double, 4, 1> q = state->_imu->quat();
  Eigen::Matrix<double, 3, 3> qToSO3 = quat_2_Rot(q);
  Eigen::Matrix<double, 3, 3> SO3Toso3 = qToSO3.log();
  Eigen::Matrix<double, 3, 1> w = Eigen::Matrix<double, 3, 1>::Zero();
  w(0) = SO3Toso3(2, 1);
  w(1) = SO3Toso3(0, 2);
  w(2) = SO3Toso3(1, 0);


  Eigen::Matrix<double, 3, 1> p = state->_imu->pos();
  Eigen::Matrix<double, 3, 1> v = state->_imu->vel();
  Eigen::Matrix<double, 3, 1> bg = state->_imu->bias_g();
  Eigen::Matrix<double, 3, 1> ba = state->_imu->bias_a();

  Eigen::Matrix<double, 15, 1> mu;
  mu << w, p, v, bg, ba;
  // std::cout << mu;
  // std::cout << "mu^^^\n";
  // should these be new or not? ^^
  sigma_points[0] = mu;
  weights[0] = (1.0*k)/(n + k);

  int i = 1;
  while (i < 16) {
    sigma_points[i] = mu + L_prime.col(i-1);
    weights[i] = 1.0/(2*(n + k));
    i++;
  }

  i = 16;
  while (i < 31) {
    sigma_points[i] = mu + L_prime.col(i-16);
    weights[i] = 1.0/(2*(n + k));
    i++;
  }

  // i = 0;
  // while (i < 31) {
  //   std::cout << weights[i];
  //   std::cout << "dkjfalkjdasfjndsa;l\n";
  //   i++;
  // }

  
  Eigen::Matrix<double, 15, 1> transformed_points [31];

  for (int i = 0; i < 31; i++) {
    transformed_points[i] = Eigen::Matrix<double, 15, 1>::Zero();
    // Eigen::Matrix<double, 7, 7> SE3_point = Eigen::Matrix<double, 7, 7>::Zero();
    // std::cout << quat_2_Rot(sigma_points[i].block(0, 0, 4, 1));
    // std::cout << "quat2rot^^\n";
    // std::cout << SE3_point.block(0, 0, 3, 3);
    // std::cout << "block test ^^\n";
    Eigen::Matrix<double, 3, 3> w_mat = Eigen::Matrix<double, 3, 3>::Zero();
    w_mat(0, 1) = -sigma_points[i](2);
    w_mat(0, 2) = sigma_points[i](1);
    w_mat(1, 0) = sigma_points[i](2);
    w_mat(1, 2) = -sigma_points[i](0);
    w_mat(2, 0) = -sigma_points[i](1);
    w_mat(2, 1) = sigma_points[i](0);
    // SE3_point.block(0, 0, 3, 3) = w_mat.exp();
    // std::cout << "line test\n";


    // SE3_point.col(3) << sigma_points[i].block(3, 0, 3, 1), 1, 0, 0, 0;
    // // std::cout << "line test2\n";
    // SE3_point.col(4) << sigma_points[i].block(6, 0, 3, 1), 0, 1, 0, 0;
    // SE3_point.col(5) << sigma_points[i].block(9, 0, 3, 1), 0, 0, 1, 0;
    // SE3_point.col(6) << sigma_points[i].block(12, 0, 3, 1), 0, 0, 0, 1;
    // std::cout << SE3_point;
    // std::cout << "SE3_point^^\n";
    Eigen::Matrix<double, 7, 7> se3_point = Eigen::Matrix<double, 7, 7>::Zero();
    se3_point.block(0, 0, 3, 3) = w_mat;
    // std::cout << w_mat;
    // std::cout << "logged^^\n";
    // std::string s;
    // std::cin >> s;
    Eigen::Matrix<double, 3, 1> w;
    w(0) = se3_point(2, 1);;
    w(1) = se3_point(0, 2);
    w(2) = se3_point(1, 0);



    double theta = w.norm();
    Eigen::Matrix<double, 3, 3> J = Eigen::Matrix<double, 3, 3>::Identity() + ((1 - cos(theta))/(theta*theta))*se3_point.block(0, 0, 3, 3) + ((theta - sin(theta))/(theta*theta*theta))*(se3_point.block(0, 0, 3, 3)*se3_point.block(0, 0, 3, 3));
    se3_point.block(0, 3, 3, 1) = (J.inverse())*sigma_points[i].block(3, 0, 3, 1);
    se3_point.block(0, 4, 3, 1) = (J.inverse())*sigma_points[i].block(6, 0, 3, 1);
    se3_point.block(0, 5, 3, 1) = (J.inverse())*sigma_points[i].block(9, 0, 3, 1);
    se3_point.block(0, 6, 3, 1) = (J.inverse())*sigma_points[i].block(12, 0, 3, 1);

    transformed_points[i](0) = w(0);
    transformed_points[i](1) = w(1);
    transformed_points[i](2) = w(2);
    transformed_points[i].block(3, 0, 3, 1) = se3_point.col(3).block(0, 0, 3, 1);
    transformed_points[i].block(6, 0, 3, 1) = se3_point.col(4).block(0, 0, 3, 1);
    transformed_points[i].block(9, 0, 3, 1) = se3_point.col(5).block(0, 0, 3, 1);
    transformed_points[i].block(12, 0, 3, 1) = se3_point.col(6).block(0, 0, 3, 1);
    // std::cout << se3_point;
    // std::cout << "se3_point^^\n";
    // std::cout << transformed_points[i];
    // std::cout << "twist^^\n";


  }

  Eigen::Matrix<double, 15, 1> mu_prime = Eigen::Matrix<double, 15, 1>::Zero();
  Eigen::Matrix<double, 15, 15> Qd_prime = Eigen::Matrix<double, 15, 15>::Zero();



  for (int i = 0; i < 31; i++) {
    // std::cout << i;
    // std::cout << "i^^\n";
    // std::cout << weights[i];
    // std::cout << "weight^^\n";
    // std::cout << transformed_points[i];
    // std::cout << "transformed_points[i]^^\n";
    // std::cout << mu_prime;
    // std::cout << "mu_prime in loop^^\n";
    mu_prime = mu_prime + weights[i]*transformed_points[i];
  }

  for (int i = 0; i < 31; i++) {
    Qd_prime = Qd_prime + weights[i]*((transformed_points[i] - mu_prime)*(transformed_points[i] - mu_prime).transpose());
  }

    // std::cout << mu_prime;
    // std::cout << "mu_prime^^\n";
    // std::cout << Qd_prime;
    // std::cout << "Qd_prime^^\n";



  //inEKF state transition calculation
  Eigen::Matrix<double, 3, 3> w_hat_mat = Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, 7, 7> se3_mu_prime = Eigen::Matrix<double, 7, 7>::Zero();
  w_hat_mat(0, 1) = -mu_prime(2);
  w_hat_mat(0, 2) = mu_prime(1);
  w_hat_mat(1, 0) = mu_prime(2);
  w_hat_mat(1, 2) = -mu_prime(0);
  w_hat_mat(2, 0) = -mu_prime(1);
  w_hat_mat(2, 1) = mu_prime(0);
  se3_mu_prime.block(0, 0, 3, 3) = w_hat_mat;
  se3_mu_prime.col(1) << mu_prime.block(3, 0, 3, 1), 0, 0, 0, 0;
  se3_mu_prime.col(2) << mu_prime.block(6, 0, 3, 1), 0, 0, 0, 0;
  se3_mu_prime.col(3) << mu_prime.block(9, 0, 3, 1), 0, 0, 0, 0;
  se3_mu_prime.col(4) << mu_prime.block(12, 0, 3, 1), 0, 0, 0, 0;

  //  Eigen::Matrix<double, 7, 7> SE3_mu_prime = se3_mu_prime.exp();

  Eigen::MatrixXd A = Eigen::Matrix<double, 15, 15>::Zero();
  w_hat_mat = Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, 3, 3> a_hat_mat = Eigen::Matrix<double, 3, 3>::Zero();

  if (state->_options.imu_avg) {
    w_hat = .5 * (w_hat + w_hat2);
    a_hat = .5 * (a_hat + a_hat2);
  }
  

  w_hat_mat(0, 1) = -w_hat(2);
  w_hat_mat(0, 2) = w_hat(1);
  w_hat_mat(1, 0) = w_hat(2);
  w_hat_mat(1, 2) = -w_hat(0);
  w_hat_mat(2, 0) = -w_hat(1);
  w_hat_mat(2, 1) = w_hat(0);

  a_hat_mat(0, 1) = -a_hat(2);
  a_hat_mat(0, 2) = a_hat(1);
  a_hat_mat(1, 0) = a_hat(2);
  a_hat_mat(1, 2) = -a_hat(0);
  a_hat_mat(2, 0) = -a_hat(1);
  a_hat_mat(2, 1) = a_hat(0);




  A.block(0, 0, 3, 3) = -w_hat_mat;
  A.block(3, 0, 3, 3) = -a_hat_mat;
  A.block(3, 3, 3, 3) = -w_hat_mat;
  A.block(6, 6, 3, 3) = -w_hat_mat;
  A.block(6, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

  PHI = (A*dt).exp();

  // new_mu = PHI*SE3_mu_prime; // + wk?


  // new_q = rot_2_quat(new_mu.block(0, 0, 3, 3));
  // new_p = new_mu.block(0, 3, 3, 1);
  // new_v = new_mu.block(0, 6, 3, 1);

  new_mu = mu_prime;
  Qd = Qd_prime;
  














  
  // int th_id = state->_imu->q()->id() - state->_imu->id();
  // int p_id = state->_imu->p()->id() - state->_imu->id();
  // int v_id = state->_imu->v()->id() - state->_imu->id();
  // int bg_id = state->_imu->bg()->id() - state->_imu->id();
  // int ba_id = state->_imu->ba()->id() - state->_imu->id();


  // std::cout << "GO TEAM 12!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"; 

  // Now replace imu estimate and fej with propagated values
  Eigen::Matrix<double, 16, 1> imu_x = state->_imu->value();
  imu_x.block(0, 0, 4, 1) = new_q;
  imu_x.block(4, 0, 3, 1) = new_p;
  imu_x.block(7, 0, 3, 1) = new_v;
  state->_imu->set_value(imu_x);
  state->_imu->set_fej(imu_x);
}

// void Propagator::predict_and_compute(std::shared_ptr<State> state, const ov_core::ImuData &data_minus, const ov_core::ImuData &data_plus,
//                                           Eigen::Matrix<double, 15, 15> &F, Eigen::Matrix<double, 15, 15> &Qd) {

//   // Set them to zero
//   F.setZero();
//   Qd.setZero();

//   // Time elapsed over interval
//   double dt = data_plus.timestamp - data_minus.timestamp;
//   // assert(data_plus.timestamp>data_minus.timestamp);

//   // Corrected imu measurements
//   Eigen::Matrix<double, 3, 1> w_hat = data_minus.wm - state->_imu->bias_g();
//   Eigen::Matrix<double, 3, 1> a_hat = data_minus.am - state->_imu->bias_a();
//   Eigen::Matrix<double, 3, 1> w_hat2 = data_plus.wm - state->_imu->bias_g();
//   Eigen::Matrix<double, 3, 1> a_hat2 = data_plus.am - state->_imu->bias_a();

//   // Compute the new state mean value
//   Eigen::Vector4d new_q;
//   Eigen::Vector3d new_v, new_p;
//   if (state->_options.use_rk4_integration)
//     predict_mean_rk4(state, dt, w_hat, a_hat, w_hat2, a_hat2, new_q, new_v, new_p);
//   else
//     predict_mean_discrete(state, dt, w_hat, a_hat, w_hat2, a_hat2, new_q, new_v, new_p);

//   // Get the locations of each entry of the imu state
//   int th_id = state->_imu->q()->id() - state->_imu->id();
//   int p_id = state->_imu->p()->id() - state->_imu->id();
//   int v_id = state->_imu->v()->id() - state->_imu->id();
//   int bg_id = state->_imu->bg()->id() - state->_imu->id();
//   int ba_id = state->_imu->ba()->id() - state->_imu->id();

//   // Allocate noise Jacobian
//   Eigen::Matrix<double, 15, 12> G = Eigen::Matrix<double, 15, 12>::Zero();

//   // Now compute Jacobian of new state wrt old state and noise
//   if (state->_options.do_fej) {

//     // This is the change in the orientation from the end of the last prop to the current prop
//     // This is needed since we need to include the "k-th" updated orientation information
//     Eigen::Matrix<double, 3, 3> Rfej = state->_imu->Rot_fej();
//     Eigen::Matrix<double, 3, 3> dR = quat_2_Rot(new_q) * Rfej.transpose();

//     Eigen::Matrix<double, 3, 1> v_fej = state->_imu->vel_fej();
//     Eigen::Matrix<double, 3, 1> p_fej = state->_imu->pos_fej();

//     F.block(th_id, th_id, 3, 3) = dR;
//     F.block(th_id, bg_id, 3, 3).noalias() = -dR * Jr_so3(-w_hat * dt) * dt;
//     // F.block(th_id, bg_id, 3, 3).noalias() = -dR * Jr_so3(-log_so3(dR)) * dt;
//     F.block(bg_id, bg_id, 3, 3).setIdentity();
//     F.block(v_id, th_id, 3, 3).noalias() = -skew_x(new_v - v_fej + _gravity * dt) * Rfej.transpose();
//     // F.block(v_id, th_id, 3, 3).noalias() = -Rfej.transpose() * skew_x(Rfej*(new_v-v_fej+_gravity*dt));
//     F.block(v_id, v_id, 3, 3).setIdentity();
//     F.block(v_id, ba_id, 3, 3) = -Rfej.transpose() * dt;
//     F.block(ba_id, ba_id, 3, 3).setIdentity();
//     F.block(p_id, th_id, 3, 3).noalias() = -skew_x(new_p - p_fej - v_fej * dt + 0.5 * _gravity * dt * dt) * Rfej.transpose();
//     // F.block(p_id, th_id, 3, 3).noalias() = -0.5 * Rfej.transpose() * skew_x(2*Rfej*(new_p-p_fej-v_fej*dt+0.5*_gravity*dt*dt));
//     F.block(p_id, v_id, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dt;
//     F.block(p_id, ba_id, 3, 3) = -0.5 * Rfej.transpose() * dt * dt;
//     F.block(p_id, p_id, 3, 3).setIdentity();

//     G.block(th_id, 0, 3, 3) = -dR * Jr_so3(-w_hat * dt) * dt;
//     // G.block(th_id, 0, 3, 3) = -dR * Jr_so3(-log_so3(dR)) * dt;
//     G.block(v_id, 3, 3, 3) = -Rfej.transpose() * dt;
//     G.block(p_id, 3, 3, 3) = -0.5 * Rfej.transpose() * dt * dt;
//     G.block(bg_id, 6, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
//     G.block(ba_id, 9, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

//   } else {

//     Eigen::Matrix<double, 3, 3> R_Gtoi = state->_imu->Rot();

//     F.block(th_id, th_id, 3, 3) = exp_so3(-w_hat * dt);
//     F.block(th_id, bg_id, 3, 3).noalias() = -exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
//     F.block(bg_id, bg_id, 3, 3).setIdentity();
//     F.block(v_id, th_id, 3, 3).noalias() = -R_Gtoi.transpose() * skew_x(a_hat * dt);
//     F.block(v_id, v_id, 3, 3).setIdentity();
//     F.block(v_id, ba_id, 3, 3) = -R_Gtoi.transpose() * dt;
//     F.block(ba_id, ba_id, 3, 3).setIdentity();
//     F.block(p_id, th_id, 3, 3).noalias() = -0.5 * R_Gtoi.transpose() * skew_x(a_hat * dt * dt);
//     F.block(p_id, v_id, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dt;
//     F.block(p_id, ba_id, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
//     F.block(p_id, p_id, 3, 3).setIdentity();

//     G.block(th_id, 0, 3, 3) = -exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
//     G.block(v_id, 3, 3, 3) = -R_Gtoi.transpose() * dt;
//     G.block(p_id, 3, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
//     G.block(bg_id, 6, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
//     G.block(ba_id, 9, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
//   }

//   // Construct our discrete noise covariance matrix
//   // Note that we need to convert our continuous time noises to discrete
//   // Equations (129) amd (130) of Trawny tech report
//   Eigen::Matrix<double, 12, 12> Qc = Eigen::Matrix<double, 12, 12>::Zero();
//   Qc.block(0, 0, 3, 3) = _noises.sigma_w_2 / dt * Eigen::Matrix<double, 3, 3>::Identity();
//   Qc.block(3, 3, 3, 3) = _noises.sigma_a_2 / dt * Eigen::Matrix<double, 3, 3>::Identity();
//   Qc.block(6, 6, 3, 3) = _noises.sigma_wb_2 * dt * Eigen::Matrix<double, 3, 3>::Identity();
//   Qc.block(9, 9, 3, 3) = _noises.sigma_ab_2 * dt * Eigen::Matrix<double, 3, 3>::Identity();

//   // Compute the noise injected into the state over the interval
//   Qd = G * Qc * G.transpose();
//   Qd = 0.5 * (Qd + Qd.transpose());

//   // Now replace imu estimate and fej with propagated values
//   Eigen::Matrix<double, 16, 1> imu_x = state->_imu->value();
//   imu_x.block(0, 0, 4, 1) = new_q;
//   imu_x.block(4, 0, 3, 1) = new_p;
//   imu_x.block(7, 0, 3, 1) = new_v;
//   state->_imu->set_value(imu_x);
//   state->_imu->set_fej(imu_x);
// }

void Propagator::predict_mean_discrete(std::shared_ptr<State> state, double dt, const Eigen::Vector3d &w_hat1,
                                       const Eigen::Vector3d &a_hat1, const Eigen::Vector3d &w_hat2, const Eigen::Vector3d &a_hat2,
                                       Eigen::Vector4d &new_q, Eigen::Vector3d &new_v, Eigen::Vector3d &new_p) {

  // If we are averaging the IMU, then do so
  Eigen::Vector3d w_hat = w_hat1;
  Eigen::Vector3d a_hat = a_hat1;
  if (state->_options.imu_avg) {
    w_hat = .5 * (w_hat1 + w_hat2);
    a_hat = .5 * (a_hat1 + a_hat2);
  }

  // Pre-compute things
  double w_norm = w_hat.norm();
  Eigen::Matrix<double, 4, 4> I_4x4 = Eigen::Matrix<double, 4, 4>::Identity();
  Eigen::Matrix<double, 3, 3> R_Gtoi = state->_imu->Rot();

  // Orientation: Equation (101) and (103) and of Trawny indirect TR
  Eigen::Matrix<double, 4, 4> bigO;
  if (w_norm > 1e-20) {
    bigO = cos(0.5 * w_norm * dt) * I_4x4 + 1 / w_norm * sin(0.5 * w_norm * dt) * Omega(w_hat);
  } else {
    bigO = I_4x4 + 0.5 * dt * Omega(w_hat);
  }
  new_q = quatnorm(bigO * state->_imu->quat());
  // new_q = rot_2_quat(exp_so3(-w_hat*dt)*R_Gtoi);

  // Velocity: just the acceleration in the local frame, minus global gravity
  new_v = state->_imu->vel() + R_Gtoi.transpose() * a_hat * dt - _gravity * dt;

  // Position: just velocity times dt, with the acceleration integrated twice
  new_p = state->_imu->pos() + state->_imu->vel() * dt + 0.5 * R_Gtoi.transpose() * a_hat * dt * dt - 0.5 * _gravity * dt * dt;
}

void Propagator::predict_mean_rk4(std::shared_ptr<State> state, double dt, const Eigen::Vector3d &w_hat1, const Eigen::Vector3d &a_hat1,
                                  const Eigen::Vector3d &w_hat2, const Eigen::Vector3d &a_hat2, Eigen::Vector4d &new_q,
                                  Eigen::Vector3d &new_v, Eigen::Vector3d &new_p) {

  // Pre-compute things
  Eigen::Vector3d w_hat = w_hat1;
  Eigen::Vector3d a_hat = a_hat1;
  Eigen::Vector3d w_alpha = (w_hat2 - w_hat1) / dt;
  Eigen::Vector3d a_jerk = (a_hat2 - a_hat1) / dt;

  // y0 ================
  Eigen::Vector4d q_0 = state->_imu->quat();
  Eigen::Vector3d p_0 = state->_imu->pos();
  Eigen::Vector3d v_0 = state->_imu->vel();

  // k1 ================
  Eigen::Vector4d dq_0 = {0, 0, 0, 1};
  Eigen::Vector4d q0_dot = 0.5 * Omega(w_hat) * dq_0;
  Eigen::Vector3d p0_dot = v_0;
  Eigen::Matrix3d R_Gto0 = quat_2_Rot(quat_multiply(dq_0, q_0));
  Eigen::Vector3d v0_dot = R_Gto0.transpose() * a_hat - _gravity;

  Eigen::Vector4d k1_q = q0_dot * dt;
  Eigen::Vector3d k1_p = p0_dot * dt;
  Eigen::Vector3d k1_v = v0_dot * dt;

  // k2 ================
  w_hat += 0.5 * w_alpha * dt;
  a_hat += 0.5 * a_jerk * dt;

  Eigen::Vector4d dq_1 = quatnorm(dq_0 + 0.5 * k1_q);
  // Eigen::Vector3d p_1 = p_0+0.5*k1_p;
  Eigen::Vector3d v_1 = v_0 + 0.5 * k1_v;

  Eigen::Vector4d q1_dot = 0.5 * Omega(w_hat) * dq_1;
  Eigen::Vector3d p1_dot = v_1;
  Eigen::Matrix3d R_Gto1 = quat_2_Rot(quat_multiply(dq_1, q_0));
  Eigen::Vector3d v1_dot = R_Gto1.transpose() * a_hat - _gravity;

  Eigen::Vector4d k2_q = q1_dot * dt;
  Eigen::Vector3d k2_p = p1_dot * dt;
  Eigen::Vector3d k2_v = v1_dot * dt;

  // k3 ================
  Eigen::Vector4d dq_2 = quatnorm(dq_0 + 0.5 * k2_q);
  // Eigen::Vector3d p_2 = p_0+0.5*k2_p;
  Eigen::Vector3d v_2 = v_0 + 0.5 * k2_v;

  Eigen::Vector4d q2_dot = 0.5 * Omega(w_hat) * dq_2;
  Eigen::Vector3d p2_dot = v_2;
  Eigen::Matrix3d R_Gto2 = quat_2_Rot(quat_multiply(dq_2, q_0));
  Eigen::Vector3d v2_dot = R_Gto2.transpose() * a_hat - _gravity;

  Eigen::Vector4d k3_q = q2_dot * dt;
  Eigen::Vector3d k3_p = p2_dot * dt;
  Eigen::Vector3d k3_v = v2_dot * dt;

  // k4 ================
  w_hat += 0.5 * w_alpha * dt;
  a_hat += 0.5 * a_jerk * dt;

  Eigen::Vector4d dq_3 = quatnorm(dq_0 + k3_q);
  // Eigen::Vector3d p_3 = p_0+k3_p;
  Eigen::Vector3d v_3 = v_0 + k3_v;

  Eigen::Vector4d q3_dot = 0.5 * Omega(w_hat) * dq_3;
  Eigen::Vector3d p3_dot = v_3;
  Eigen::Matrix3d R_Gto3 = quat_2_Rot(quat_multiply(dq_3, q_0));
  Eigen::Vector3d v3_dot = R_Gto3.transpose() * a_hat - _gravity;

  Eigen::Vector4d k4_q = q3_dot * dt;
  Eigen::Vector3d k4_p = p3_dot * dt;
  Eigen::Vector3d k4_v = v3_dot * dt;

  // y+dt ================
  Eigen::Vector4d dq = quatnorm(dq_0 + (1.0 / 6.0) * k1_q + (1.0 / 3.0) * k2_q + (1.0 / 3.0) * k3_q + (1.0 / 6.0) * k4_q);
  new_q = quat_multiply(dq, q_0);
  new_p = p_0 + (1.0 / 6.0) * k1_p + (1.0 / 3.0) * k2_p + (1.0 / 3.0) * k3_p + (1.0 / 6.0) * k4_p;
  new_v = v_0 + (1.0 / 6.0) * k1_v + (1.0 / 3.0) * k2_v + (1.0 / 3.0) * k3_v + (1.0 / 6.0) * k4_v;
}
