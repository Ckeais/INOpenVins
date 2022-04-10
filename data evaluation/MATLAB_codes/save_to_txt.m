clc;close all;clear all;

A = readtable('groundtruth_outdoor5.txt');
load('original_ov_indoor_forward_5_dragon_pathimu.mat')
load('team_12_indoor_forward_5_dragon_pathimu.mat')

t_gt = (A.Var1 - A.Var1(1))+20.47;
t_0_ours = outdoor5_ours(1,1)+outdoor5_ours(1,2) * 10^-9;
t_ours = outdoor5_ours(:,1)+ outdoor5_ours(:,2) .* 10^-9 - t_0_ours;
t_0_openvins = outdoor5_openvins(1,1)+outdoor5_openvins(1,2) * 10^-9;
t_openvins = outdoor5_openvins(:,1)+outdoor5_openvins(:,2) * 10^-9 - t_0_openvins +1 ;

% extract processing
t_end = t_gt(length(t_gt));
t_start = 20.47;
outdoor5_ours = outdoor5_ours((t_ours>t_start&t_ours<t_end),:);
outdoor5_openvins = outdoor5_openvins((t_openvins>t_start&t_openvins<t_end),:);
t_openvins = t_openvins((t_openvins>t_start&t_openvins<t_end));
t_ours = t_ours((t_ours>t_start&t_ours<t_end));


result_outdoor_5_gt = [t_gt,A.Var2,A.Var3,A.Var4,A.Var5,A.Var6,A.Var7,A.Var8];
result_outdoor_5_openvins = [t_openvins,outdoor5_openvins(:,3:9)];
result_outdoor_5 = [t_ours,outdoor5_ours(:,3:9)];

% dlmwrite('indoor_5_gt',result_outdoor_5_gt,'delimiter',' ')
% dlmwrite('indoor_5_ours',result_outdoor_5,'delimiter',' ')
% dlmwrite('indoor_5_openvins',result_outdoor_5_openvins,'delimiter',' ')

%% tranform gt to match estimated
rotm_gt = quat2rotm([result_outdoor_5_gt(1,8),result_outdoor_5_gt(1,5),result_outdoor_5_gt(1,6),result_outdoor_5_gt(1,7)]);
rotm_ours = quat2rotm([result_outdoor_5(1,8),result_outdoor_5(1,5),result_outdoor_5(1,6),result_outdoor_5(1,7)]);
R = rotm_gt * rotm_ours';
t = [result_outdoor_5_gt(1,2);result_outdoor_5_gt(1,3);result_outdoor_5_gt(1,4)] - R * [result_outdoor_5(1,2);result_outdoor_5(1,3);result_outdoor_5(1,4)];
for i = 1:size(result_outdoor_5_gt,1)
  gt_xyz(i,:) = [R',-R'*t;zeros(1,3),1] * [result_outdoor_5_gt(i,2);result_outdoor_5_gt(i,3);result_outdoor_5_gt(i,4);1];
end

% assign the transfromed gt to the output
result_outdoor_5_gt(:,2:4) = gt_xyz(:,1:3);

%% saving data as txt file
dlmwrite('indoor_5_gt',result_outdoor_5_gt,'delimiter',' ')
dlmwrite('indoor_5_ours',result_outdoor_5,'delimiter',' ')
dlmwrite('indoor_5_openvins',result_outdoor_5_openvins,'delimiter',' ')

%% ploting the figure
figure
hold on
plot3(gt_xyz(:,1),gt_xyz(:,2),gt_xyz(:,3),'LineWidth',2)
plot3(result_outdoor_5(:,2),result_outdoor_5(:,3),result_outdoor_5(:,4),'LineWidth',2)
plot3(result_outdoor_5_openvins(:,2),result_outdoor_5_openvins(:,3),result_outdoor_5_openvins(:,4),'LineWidth',2)
hold off
view([20,20])
legend('ground truth','OpenVINS Modified','OpenVINS')
grid on

figure
subplot(3,1,1)
hold on
plot(t_gt,gt_xyz(:,1),'LineWidth',2)
plot(t_ours,result_outdoor_5(:,2),'LineWidth',2)
plot(t_openvins,result_outdoor_5_openvins(:,2),'LineWidth',2)
hold off
legend('ground truth','OpenVINS Modified','OpenVINS')
subplot(3,1,2)
hold on
plot(t_gt,gt_xyz(:,2),'LineWidth',2)
plot(t_ours,result_outdoor_5(:,3),'LineWidth',2)
plot(t_openvins,result_outdoor_5_openvins(:,3),'LineWidth',2)
hold off
subplot(3,1,3)
hold on
plot(t_gt,gt_xyz(:,3),'LineWidth',2)
plot(t_ours,result_outdoor_5(:,4),'LineWidth',2)
plot(t_openvins,result_outdoor_5_openvins(:,4),'LineWidth',2)
hold off




