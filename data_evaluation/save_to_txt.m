clc;close all;clear all;

% loading data (change the name for different datasets)
gt = readtable('groundtruth_outdoor9.txt');
load('original_ov_indoor_forward_9_dragon_pathimu.mat')
load('team_12_indoor_forward_9_dragon_pathimu.mat')
ours  = outdoor9_ours;
openvins = outdoor9_openvins;


%% extract lineuped data
t_gt = (gt.Var1 - gt.Var1(1)) + 20.5; % the time is manually determined
t_0_ours = ours(1,1)+ours(1,2) * 10^-9;
t_ours = ours(:,1)+ ours(:,2) .* 10^-9 - t_0_ours;
t_0_openvins = openvins(1,1)+openvins(1,2) * 10^-9;
t_openvins = openvins(:,1)+openvins(:,2) * 10^-9 - t_0_openvins +9.5;

t_end = t_gt(length(t_gt));
t_start = 20.47;
ours = ours((t_ours>t_start&t_ours<t_end),:);
openvins = openvins((t_openvins>t_start&t_openvins<t_end),:);
t_openvins = t_openvins((t_openvins>t_start&t_openvins<t_end));
t_ours = t_ours((t_ours>t_start&t_ours<t_end));

result_gt = [t_gt,gt.Var2,gt.Var3,gt.Var4,gt.Var5,gt.Var6,gt.Var7,gt.Var8];
result_openvins = [t_openvins,openvins(:,3:9)];
result_ours = [t_ours,ours(:,3:9)];


%% tranform gt to match estimated
rotm_gt = quat2rotm([result_gt(1,8),result_gt(1,5),result_gt(1,6),result_gt(1,7)]);
rotm_ours = quat2rotm([result_ours(1,8),result_ours(1,5),result_ours(1,6),result_ours(1,7)]);
R = rotm_gt * rotm_ours';
t = [result_gt(1,2);result_gt(1,3);result_gt(1,4)] - R' * [result_ours(1,2);result_ours(1,3);result_ours(1,4)];
for i = 1:size(result_gt,1)
  gt_xyz(i,:) = [R',-R'*t;zeros(1,3),1] * [result_gt(i,2);result_gt(i,3);result_gt(i,4);1];
end



% assign the transfromed gt to the output
%result_gt(:,2:4) = gt_xyz(:,1:3);

%% saving data as txt file
dlmwrite('indoor_9_gt.txt',result_gt,'delimiter',' ')
dlmwrite('indoor_9_ours.txt',result_ours,'delimiter',' ')
dlmwrite('indoor_9_openvins.txt',result_openvins,'delimiter',' ')

%% ploting the figure
% ploting unmatched data
figure
hold on
plot3(gt.Var2,gt.Var3,gt.Var4,'LineWidth',2)
plot3(gt_xyz(:,1),gt_xyz(:,2),gt_xyz(:,3),'LineWidth',2)
plot3(ours(:,3),ours(:,4),ours(:,5),'LineWidth',2)
plot3(openvins(:,3),openvins(:,4),openvins(:,5),'LineWidth',2)
hold off
view([20,20])
legend('ground truth','gt matched','OpenVINS Modified','OpenVINS')
grid on

% plot unmatched x y z
figure
subplot(3,1,1)
hold on
plot(t_gt,gt.Var2,'LineWidth',2)
plot(t_gt,gt_xyz(:,1),'LineWidth',2)
plot(t_ours,ours(:,3),'LineWidth',2)
plot(t_openvins,openvins(:,3),'LineWidth',2)
hold off
legend('ground truth','gt_matched','OpenVINS Modified','OpenVINS')
subplot(3,1,2)
hold on
plot(t_gt,gt.Var3,'LineWidth',2)
plot(t_gt,gt_xyz(:,2),'LineWidth',2)
plot(t_ours,ours(:,4),'LineWidth',2)
plot(t_openvins,openvins(:,4),'LineWidth',2)
hold off
subplot(3,1,3)
hold on
plot(t_gt,gt.Var4,'LineWidth',2)
plot(t_gt,gt_xyz(:,2),'LineWidth',2)
plot(t_ours,ours(:,5),'LineWidth',2)
plot(t_openvins,openvins(:,5),'LineWidth',2)
hold off

% figure
% hold on
% plot3(result_gt(:,2),result_gt(:,3),result_gt(:,4),'LineWidth',2)
% plot3(openvins_xyz(:,1),openvins_xyz(:,2),openvins_xyz(:,3),'LineWidth',2)
% plot3(ours_xyz(:,1),ours_xyz(:,2),ours_xyz(:,3),'LineWidth',2)
% hold off
% view([20,20])
% legend('ground truth','OpenVINS','OpenVINS Modified')
% grid on

% figure
% hold on
% plot3(result_gt(:,2),result_gt(:,3),result_gt(:,4))
% plot3(result_ours(:,2),result_ours(:,3),result_ours(:,4))
% plot3(result_openvins(:,2),result_openvins(:,3),result_openvins(:,4))
% hold off




