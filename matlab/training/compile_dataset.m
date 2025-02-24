close all; clear all; clc;
addpath('./util')

map = brewermap(9,'Set1');

%% Load in data

%The mass of the arm with nothing on it
m0 = 583;

%in respective order: The socket alone, the brush alone, mass C + brush, 
%                       mass A + brush, mass A + mass C + brush 
mass = m0+[48,146,146+250,146+580,146+255+580]; 
mass = (mass/1000)*9.81;
y_home = [0.3209,0.3177,0.3164,0.3153,0.3144];


mass_vis = 3; % Default is -1 for all 

root_path = "./data/";
data_path = ["-50g_random/2025_02_03_12_52_20","0g_random/2025_02_02_16_29_02","250g_random/2025_01_29_11_38_12","500gA_random/2025_01_30_11_12_34","750g_random/2025_01_31_09_52_51"]; %,"1000gAB\2024_11_09_16_32_42"];
file_path = "/positions.csv";

colors = lines(7);

% Pull in first data to initialize the headers
T_full = readtable(root_path+data_path(1)+file_path,"Range",[1,4]);

%% Save dataset with only new data

plot_data = false;

header = [T_full.Properties.VariableNames(1:7), {'f_t'}, T_full.Properties.VariableNames(8:16)];
T_new = [];

if plot_data
    figure(1); clf; hold on; grid on;
end

% Pull in recent data
for trial = 1:length(mass) 
    T_i = readtable(root_path+data_path(trial)+file_path,"Range",[1,4]);
    T_i.y_end_avg = T_i.y_end_avg-T_i.y_end_avg(1) + y_home(trial);
    disp(size(T_i))
    % Align quaternions with [1,0,0,0] home position
    for i = 1:(size(T_i,1))/100
        T_i_snip = T_i((i-1)*100+1:i*100,:);
        T_i_snip_new = batch_quaternion_transform(T_i_snip);
        T_new = [T_new;[table2array(T_i_snip_new(:,1:7)),repmat(mass(trial),[size(T_i_snip_new,1),1]),table2array(T_i_snip_new(:,8:16))]];
    end

end

% Create table
T_new = array2table(T_new,"VariableNames",header);

% Remove any mid-flip data points
q_slice = T_new.qw_end_avg > 0.8;
T_new = T_new(q_slice,:);

writetable(T_new, 'data/feb4_training_data_rehomed.csv');

if plot_data
    
    % Define legend labels
    legendLabels = ["500 g", "527 g", "655 g", "978 g", "1233 g", "1542 g"];
    
    % Add labels, legend, and formatting
    xlabel('X-axis (mm)');
    ylabel('Y-axis (mm)');
    zlabel('Z-axis (mm)');
    title('Point Cloud Visualization');
    legend(legendLabels, 'Location', 'bestoutside');
end

%% 
% Extract positions from the table
x = T_i_snip.x_end_avg;
y = T_i_snip.y_end_avg;
z = T_i_snip.z_end_avg;

% Extract quaternion components (assumed [w, x, y, z] order)
qw = T_i_snip.qw_end_avg;
qx = T_i_snip.qx_end_avg;
qy = T_i_snip.qy_end_avg;
qz = T_i_snip.qz_end_avg;

% Create a new figure
figure; 
hold on; 
grid on; 
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Orientation Triads for 100-Point Snippet');

% Plot the position points
plot3(x, y, z, 'ko', 'MarkerFaceColor', 'k');

% Define a scaling factor for the triad vectors
scale = 0.01;  % adjust as needed for visualization

% Loop through each point and plot the triad
for k = 1:height(T_i_snip_new)
    % Build quaternion for the current point
    quat = [qw(k), qx(k), qy(k), qz(k)];
    
    % Convert quaternion to rotation matrix (each column is one axis vector)
    R = quat2rotm(quat);
    
    % Extract the axes directions from the rotation matrix
    x_axis = R(:,1);
    y_axis = R(:,2);
    z_axis = R(:,3);
    
    % Position for this point
    p = [x(k), y(k), z(k)];
    
    % Plot triad arrows using quiver3:
    quiver3(p(1), p(2), p(3), scale*x_axis(1), scale*x_axis(2), scale*x_axis(3), ...
            'r', 'LineWidth', 1.5, 'MaxHeadSize', 1);
    quiver3(p(1), p(2), p(3), scale*y_axis(1), scale*y_axis(2), scale*y_axis(3), ...
            'g', 'LineWidth', 1.5, 'MaxHeadSize', 1);
    quiver3(p(1), p(2), p(3), scale*z_axis(1), scale*z_axis(2), scale*z_axis(3), ...
            'b', 'LineWidth', 1.5, 'MaxHeadSize', 1);
end

hold off;
%% 
% Extract positions from the table
x = T_i_snip_new.x_end_avg;
y = T_i_snip_new.y_end_avg;
z = T_i_snip_new.z_end_avg;

% Extract quaternion components (assumed [w, x, y, z] order)
qw = T_i_snip_new.qw_end_avg;
qx = T_i_snip_new.qx_end_avg;
qy = T_i_snip_new.qy_end_avg;
qz = T_i_snip_new.qz_end_avg;

% Create a new figure
figure; 
hold on; 
grid on; 
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Orientation Triads for 100-Point Snippet');

% Plot the position points
plot3(x, y, z, 'ko', 'MarkerFaceColor', 'k');

% Define a scaling factor for the triad vectors
scale = 0.01;  % adjust as needed for visualization

% Loop through each point and plot the triad
for k = 1:height(T_i_snip_new)
    % Build quaternion for the current point
    quat = [qw(k), qx(k), qy(k), qz(k)];
    
    % Convert quaternion to rotation matrix (each column is one axis vector)
    R = quat2rotm(quat);
    
    % Extract the axes directions from the rotation matrix
    x_axis = R(:,1);
    y_axis = R(:,2);
    z_axis = R(:,3);
    
    % Position for this point
    p = [x(k), y(k), z(k)];
    
    % Plot triad arrows using quiver3:
    quiver3(p(1), p(2), p(3), scale*x_axis(1), scale*x_axis(2), scale*x_axis(3), ...
            'r', 'LineWidth', 1.5, 'MaxHeadSize', 1);
    quiver3(p(1), p(2), p(3), scale*y_axis(1), scale*y_axis(2), scale*y_axis(3), ...
            'g', 'LineWidth', 1.5, 'MaxHeadSize', 1);
    quiver3(p(1), p(2), p(3), scale*z_axis(1), scale*z_axis(2), scale*z_axis(3), ...
            'b', 'LineWidth', 1.5, 'MaxHeadSize', 1);
end

hold off;

%% Plot data

%%%%%%%%%%%%%
%NOTE Currently all data in the compiled csv will have the positions in
%meters.
%Turn positions to csv daa
T_csv = T_full(:,4:19);
T_csv.mass = repmat(mass(1), height(T_csv), 1); % Repeat mass(1) for the number of rows in T_csv

T_left = T_csv(:,1:7);
T_right = T_csv(:,8:end-1);
T_mid = T_csv(:,end);
T_csv_original_data = [T_left,T_mid,T_right];
%writetable(T_csv_original_data, 'data/original_training_data.csv');

 % Add point cloud
 figure(1); clf; hold on; grid on;

 % Define legend labels
 legendLabels = ["450", "500", "527", "655", "978", "1233", "1542"];


 T_full_subA = table2array(T_full_sub)*1000;
 T_full_subA(:,8) = mass(1);
 s = scatter3(T_full_subA(1:1500,1),T_full_subA(1:1500,2),T_full_subA(1:1500,3),'filled', 'MarkerFaceColor', colors(1,:), 'MarkerEdgeColor','k','LineWidth',1);
 s.SizeData=5;

% Pull in recent data
for trial = 2:length(mass)
    
    if mass_vis < 0 || trial == mass_vis
    
        T_i = readtable(root_path+data_path(trial)+file_path);
        
        T_i(:,4:6) = T_i(:,4:6) - T_i(1,4:6);
        T_i_sub = T_i(:,4:10);
        
        %Creating table for the csv of each iteration.
        T_i_csv = T_i(:,4:19);
        T_i_csv(:,1:7) = T_i_sub;
        T_i_csv.mass = repmat(mass(trial), height(T_i_csv), 1); % Repeat mass(1) for the number of rows in T_csv
        T_csv = vertcat(T_csv, T_i_csv);
    
        T_i_sub = table2array(T_i_sub)*1000;
        T_i_sub(:,8) = mass(trial);
    
        %Add each sub table to the full table
        T_full_subA = [T_full_subA;T_i_sub];
    
        X = T_i_sub(:,1);
        s = scatter3(T_i_sub(:,1),T_i_sub(:,2),T_i_sub(:,3),'filled', 'MarkerFaceColor', colors(trial,:), 'MarkerEdgeColor','none','LineWidth',1);
        s.SizeData=50;
        
        disp(['Total points: ', num2str(size(T_full_subA, 1))]);

    end
        
end

T_left = T_csv(:,1:7);
T_right = T_csv(:,8:end-1);
T_mid = T_csv(:,end);
T_csv_correct = [T_left,T_mid,T_right];

writetable(T_csv_correct, 'data/weighted_training_data.csv');

% Add labels, legend, and formatting
xlabel('X-axis (mm)');
ylabel('Y-axis (mm)');
zlabel('Z-axis (mm)');
title('Point Cloud Visualization');
legend(legendLabels, 'Location', 'bestoutside');


hold off