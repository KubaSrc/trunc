close all; clear all; clc;
addpath('../util')

warning('off', 'all');

home_pos = load('../state/home_measured.mat').pos;

lw = 2;
ms = 8;

%% Plotting out trajectory runs

dir_path = './mocap/8-7-2024/';
mocap_path = [dir_path,'mocap_take'];
export_wp = true;

% Human recorded data
name = 'RigidBody';
cols = {[name],[name,'_1'],[name,'_2'],[name,'_3'],[name,'_4'],[name,'_5'],[name,'_6']};
new_cols = {'qx_end_avg','qy_end_avg','qz_end_avg','qw_end_avg','x_end_avg','z_end_avg','y_end_avg'};
T = readtable(mocap_path);
T = T(:,cols);
T = rmmissing(T);
T = renamevars(T,cols,new_cols);
T = T(:, {'x_end_avg','y_end_avg','z_end_avg','qx_end_avg','qy_end_avg','qz_end_avg','qw_end_avg'});
T.y_end_avg = -T.y_end_avg;

v_names = T.Properties.VariableNames;
T_home = array2table([home_pos(1:3),home_pos(5:7),home_pos(4)],'VariableNames',v_names);

T = batch_transform(T_home,T);
T = T(1:50:end,:);
n=size(T,1);

if export_wp
    wp = table2array(T);
    save([dir_path,'trajectory.mat'],'wp')
end

figure(1); clf; hold on; grid on;

% Set the background color to white
set(gcf, 'Color', 'w');

plot3(T.x_end_avg(1:n),T.y_end_avg(1:n),T.z_end_avg(1:n),'b','LineWidth',2);

%% Robot replay
T_r = readmatrix([dir_path,'./data/positions.csv']);

plot3(1000.*T_r(1:n,3),1000.*T_r(1:n,4),1000.*T_r(1:n,5),'r','LineWidth',2)
xlabel('x');
ylabel('y');
zlabel('z');

T = readtable("../data/2024_02_03_14_22_02/positions_trim.csv");

% Extract X, Y, and Z data points
X = (T.x_end_avg).*1000; 
Y = (T.y_end_avg).*1000;
Z = (T.z_end_avg).*1000;

% Create alpha shape
shp = alphaShape(X, Y, Z);

% Plot the alpha shape
h = plot(shp);

% Set the transparency of the surface
h.FaceAlpha = 0.5; % Adjust the transparency (0 is fully transparent, 1 is fully opaque)
h.FaceColor = [0.5, 0.5, 0.5]; % Gray color
h.EdgeColor = 'None'; % Black wireframe
% h.LineStyle = '-'; % Solid lines for the wireframe

xlabel('x');
ylabel('y');
zlabel('z')

