clc; close all;clear all;

%% load ground truth and estimate trajectory
% Change this for different datasets
gt = load('indoor_5_gt.txt');    
est = load('indoor_5_ours.txt');
openvins = load('indoor_5_openvins.txt');

[P_gt_matched,P_es_matched,s_es,R_es,t_es]= getMatched(gt,est);
[P_gt_matched_openvins,P_es_matched_openvins,s_es_openvins,R_es_openvins,t_es_openvins]= getMatched(gt,openvins);

d = figure;
plot(P_gt_matched(:,2),P_gt_matched(:,3),'k-','linewidth',1);
hold on;
plot(P_es_matched(:,2),P_es_matched(:,3),'b-','linewidth',1);
plot(P_es_matched_openvins(:,2),P_es_matched_openvins(:,3),'r-','linewidth',1)
axis equal;
grid on;
lgd = legend('ground truth','INOpenVINS','OpenVINS');
set(lgd,'FontWeight','bold','FontSize',8,'interpreter','latex','FontName','Times New Roman','Location','best');
%title('Trajecties Top View (Indoor 5)','interpreter','latex','FontSize',12,'FontName','Times New Roman');
xlabel('x [m]','interpreter','latex','FontName','Times New Roman')
ylabel('y [m]','interpreter','latex','FontName','Times New Roman')
set(d,'Units','inches','Position',[0 0 3.25 2.5],'PaperUnits','inches','PaperSize',[3.25,2.5])
exportgraphics(d,'top_view_indoor5.pdf','ContentType','vector')



%% calculate translation and rotation error
% calculate distance travel by ground truth
x_gt = P_gt_matched(:,2);
y_gt = P_gt_matched(:,3);
z_gt = P_gt_matched(:,4);
distance_travel = 0;
for i = 2:size(P_gt_matched,1)
    distance_travel = distance_travel + sqrt( (x_gt(i) - x_gt(i-1))^2 + (y_gt(i) - y_gt(i-1))^2 + (z_gt(i) - z_gt(i-1))^2 );
end
num_segment = 4;
order_plot = {};
for k = 1:num_segment
    segments(k) = distance_travel * k / num_segment;
    order_plot{k} = num2str(segments(k));
end


% our method
[error_trans,error_rot] = getTransError(P_gt_matched,P_es_matched,4,s_es,R_es,t_es,segments);

trans_error = [];
g_trans = [];
rot_error = [];
g_rot = [];
for i = 1:size(error_trans,2)
    trans_error = [trans_error;error_trans{i}'];
%     g_trans = [g_trans;repmat(segments(i),size(error_trans{i}'))]; 
    g_trans = [g_trans;repmat(string(order_plot{i}),size(error_trans{i}'))];
    
    rot_error = [rot_error;error_rot{i}'];
    g_rot = [g_rot;repmat(string(order_plot{i}),size(error_rot{i}'))];
end


% original openvins
[error_trans_openvins,error_rot_openvins] = getTransError(P_gt_matched_openvins,P_es_matched_openvins,4,s_es_openvins,R_es_openvins,t_es_openvins,segments);
trans_error_openvins = [];
g_trans_openvins = [];
rot_error_openvins = [];
g_rot_openvins = [];
for i = 1:size(error_trans_openvins,2)
    trans_error_openvins = [trans_error_openvins;error_trans_openvins{i}'];
%     g_trans_openvins = [g_trans_openvins;repmat(segments_openvins(i),size(error_trans_openvins{i}'))];  
    g_trans_openvins = [g_trans_openvins;repmat(string(order_plot{i}),size(error_trans_openvins{i}'))];
    
    rot_error_openvins = [rot_error_openvins;error_rot_openvins{i}'];
    g_rot_openvins = [g_rot_openvins;repmat(string(order_plot{i}),size(error_rot_openvins{i}'))];
end




% figure
% boxplot(trans_error,g_trans)
% 
% figure
% boxplot(rot_error,g_rot)
trans_error_order  = categorical([g_trans;g_trans_openvins],order_plot);
h1 = figure;
boxchart(trans_error_order,[trans_error;trans_error_openvins],'GroupByColor',[repmat("INOpenVINS",size(trans_error));repmat("OpenVINS",size(trans_error_openvins))],'MarkerStyle','none');
legend('Interpreter','latex','FontSize',8,'FontName','Times New Roman')
%title('Tranlation Error Boxplot (Indoor 5)','Interpreter','latex','FontSize',12,'FontName','Times New Roman')
xlabel('Distance Traveled [m]','Interpreter','latex','FontSize',10,'FontName','Times New Roman')
ylabel('Translation Error [m]','Interpreter','latex','FontSize',10,'FontName','Times New Roman')
set(h1,'Units','inches','Position',[0 0 3.25 2.5],'PaperUnits','inches','PaperSize',[3.25,2.5])
exportgraphics(h1,'trans_error_indoor5.pdf','ContentType','vector')

rot_error_order  = categorical([g_rot;g_rot_openvins],order_plot);
h2 = figure;
boxchart(rot_error_order,[rot_error;rot_error_openvins],'GroupByColor',[repmat("INOpenVINS",size(rot_error));repmat("OpenVINS",size(rot_error_openvins))],'MarkerStyle','none');
legend('Interpreter','latex','FontSize',8,'FontName','Times New Roman')
%title('Rotational Error Boxplot (Indoor 5)','Interpreter','latex','FontSize',12,'FontName','Times New Roman')
xlabel('Distance Traveled [m]','Interpreter','latex','FontSize',10,'FontName','Times New Roman')
ylabel('Rotation Error [rad/m]','Interpreter','latex','FontSize',10,'FontName','Times New Roman')
set(h2,'Units','inches','Position',[0 0 3.25 2.5],'PaperUnits','inches','PaperSize',[3.25,2.5])
exportgraphics(h2,'rot_error_indoor5.pdf','ContentType','vector')

%% draw error in XYZ axes
% errX = P_es_aligned(1,Ids_es)-P_gt(1,Ids_gt);
% errY = P_es_aligned(2,Ids_es)-P_gt(2,Ids_gt);
% errZ = P_es_aligned(3,Ids_es)-P_gt(3,Ids_gt);
% figure;
% subplot(311);
% plot(errX);
% title('position error of axis-X (m)');
% subplot(312);
% plot(errY);
% title('position error of axis-Y (m)');
% subplot(313);
% plot(errZ);
% title('position error of axis-Z (m)');
% fprintf('mean error in [X Y Z]: [%fm %fm %fm]\n',mean(errX),mean(errY),mean(errZ));
% 
% %% some printing
% errVec = P_es_aligned(1:3,Ids_es)-P_gt(1:3,Ids_gt);
% N = size(errVec,2);
% RMSE_trans = 0;
% for i = 1:N
%     RMSE_trans = RMSE_trans+norm(errVec(:,i))^2;
% end
% RMSE_trans = sqrt(RMSE_trans/N);
% fprintf('RMSE of translation is %fm\n',RMSE_trans);