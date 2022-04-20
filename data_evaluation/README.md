# Data Evaluation

## data extaction
`rosbag_to_mat.m` is used to converted the rosbag file to `.mat` file. `save_to_txt.m` is used to convert the `.mat` file to txt to be processed in `rpg_trajectory_evaluation`. For example, for the Indoor_5 dataset. the final txt file is produced are `indoor_5_gt.txt`, `indoor_5_openvins.txt` and `indoor_5_ours`.

## Data Evaluation:
The data evaluation is done with `Data_evaluation.m` and it is based on [rpg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation). The basic is idea is to divide the trajectories into sub-trajectories based on distance traveled. Then the relative translation and axis-angle rotation error is compared with the ground truth. 



## References:
Zichao Zhang, Davide Scaramuzza: A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry, IEEE/RSJ Int. Conf. Intell. Robot. Syst. (IROS), 2018.

S. Umeyama, "Least-squares estimation of transformation parameters between two point patterns," in IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. 13, no. 4, pp. 376-380, April 1991, doi: 10.1109/34.88573.
