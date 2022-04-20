clc;close all; clear all;


bag = rosbag('original_ov_indoor_forward_9_dragon_pathimu.bag');

bSel = select(bag,'Topic','/ov_msckf/pathimu');
msgStructs = readMessages(bSel,'DataFormat','struct');


msg_size = length(msgStructs);

%% pathimu
x = zeros(msg_size,1);y = zeros(msg_size,1);z = zeros(msg_size,1);
qx = zeros(msg_size,1);qy = zeros(msg_size,1);qz = zeros(msg_size,1);
qw = zeros(msg_size,1);t_s = zeros(msg_size,1);t_Ns = zeros(msg_size,1);

for i = 1:msg_size
    pose  = msgStructs{msg_size}.Poses(i).Pose; 
    t_s(i) = double(msgStructs{i}.Header.Stamp.Sec);
    t_Ns(i) = double(msgStructs{i}.Header.Stamp.Nsec);
    x(i) = pose.Position.X;
    y(i) = pose.Position.Y;
    z(i) = pose.Position.Z;
    qx(i) = pose.Orientation.X;
    qy(i) = pose.Orientation.Y;
    qz(i) = pose.Orientation.Z;
    qw(i) = pose.Orientation.W;
end


outdoor9_openvins = [t_s,t_Ns,x,y,z,qx,qy,qz,qw];


save('original_ov_indoor_forward_9_dragon_pathimu.mat','outdoor9_openvins')

plot3(x,y,z)
grid on