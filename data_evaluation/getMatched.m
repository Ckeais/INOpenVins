function [gt_match,est_match,s_es,R_es,t_es] = getMatched(gt,est)
%% find matched and align data from  
% input:
% gt: n*8 [time,x,y,z,qx,qy,qz,qw]
% est: m*8 [time,x,y,z,qx,qy,qz,qw]
time_gt = gt(:,1);
P_gt = gt(:,2:4)';
time_es = est(:,1);
P_es = est(:,2:4)';
[Ids_es, Ids_gt] = findIds (time_es, time_gt, 0.001);
[R_es, t_es, s_es] = sim3DataAlignment (P_es(:,Ids_es), P_gt(:,Ids_gt), 'vio');

%% align the estimated trajectory
P_est_aligned = s_es*R_es*P_es + repmat(t_es,1,size(P_es,2));

P_es_matched = P_est_aligned(:,Ids_es);
P_gt_matched = P_gt(:,Ids_gt);

gt_match = gt(Ids_gt,:);
gt_match(:,2:4) = P_gt_matched';
est_match = est(Ids_es,:);
est_match(:,2:4) = P_es_matched';
end

