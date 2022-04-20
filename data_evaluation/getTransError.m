function [error_trans,error_rot] = getTransError(gt,est,num_segment,s,R,t,segments)
%calculate relative translation error of gt and est position
% divide the trajectories into 4 segment
% inputs:
% gt: n*8 [time,x,y,z,qx,qy,qz,qw]
% est: n*8 [time,x,y,z,qx,qy,qz,qw]

x_gt = gt(:,2);
y_gt = gt(:,3);
z_gt = gt(:,4);
x_est = est(:,2);
y_est = est(:,3);
z_est = est(:,4);

% distance_travel = 0;
% % calculate distance travel of ground truth
% for i = 2:size(gt,1)
%     distance_travel = distance_travel + sqrt( (x_gt(i) - x_gt(i-1))^2 + (y_gt(i) - y_gt(i-1))^2 + (z_gt(i) - z_gt(i-1))^2 );
% end
% 
% for k = 1:num_segment
%     segments(k) = distance_travel * k / num_segment;
% end

dis  = 0;
segment_count = 1;
count = 1;
for j = 1:size(gt,1)-1
    dis  = dis + sqrt( (x_gt(j+1) - x_gt(j))^2 + (y_gt(j+1) - y_gt(j))^2 + (z_gt(j+1) - z_gt(j))^2 );
    
    if dis > segments(segment_count)
        error_trans{segment_count} = trans_error;
        error_rot{segment_count} = rot_error;
        count = 1;
        rot_error = [];
        trans_error = [];
        segment_count = segment_count+1;
    end
    
    R_gt = quat2rotm([gt(j,8),gt(j,5),gt(j,6),gt(j,7)]);
    R_es_dot = R * quat2rotm([est(j,8),est(j,5),est(j,6),est(j,7)]);
    temp = rotm2axang(R_gt * R_es_dot');
    rot_error(count) = temp(4);
    trans_error(count) = norm([x_gt(j);y_gt(j);z_gt(j)] - R_gt * R_es_dot' *[x_est(j);y_est(j);z_est(j)] ,2);
    count = count + 1;
    
end

error_trans{segment_count} = trans_error;
error_rot{segment_count} = rot_error;

end

