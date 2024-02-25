close all; clear all; clc;
warning('off', 'all');

%% Plotting out trajectory runs (circle)
figure(1); clf; hold on; grid on;

% DNN
exp_DNN = 'circle_2024_02_21_15_16_35';
motor_inputs = load(['./training/experiments/',exp_DNN,'/DNN_circle_trajectory_inputs.mat']).output;
wp_DNN = load(['./training/experiments/',exp_DNN,'/circle_trajectory.mat']).wp;
T_DNN = readtable(['./training/experiments/',exp_DNN,'/positions.csv']);

plot3(T_DNN.x_end_avg.*1000,T_DNN.y_end_avg.*1000,T_DNN.z_end_avg.*1000,'-^','LineWidth',1.5,'DisplayName','DNN');
plot3(wp_DNN(:,1),wp_DNN(:,2),wp_DNN(:,3),'-x','color','k','lineWidth',1.5,'DisplayName','actual')

e_dnn = sqrt(sum((wp_DNN(:,1:3)-[T_DNN.x_end_avg.*1000,T_DNN.y_end_avg.*1000,T_DNN.z_end_avg.*1000]).^2,2));

legend()

xlabel('x (mm)')
ylabel('y (mm)')
zlabel('z (mm)')



%% Plotting out trajectory runs (triangle)
figure(2); clf; hold on; grid on; axis equal;


% DNN
exp_DNN = 'triangle_2024_02_22_15_48_39';
motor_inputs = load(['./training/experiments/',exp_DNN,'/DNN_triangle_trajectory_inputs.mat']).output;
wp_DNN = load(['./training/experiments/',exp_DNN,'/triangle_trajectory.mat']).wp;
T_DNN = readtable(['./training/experiments/',exp_DNN,'/positions.csv']);

plot3(T_DNN.x_end_avg.*1000,T_DNN.y_end_avg.*1000,T_DNN.z_end_avg.*1000,'-^','LineWidth',1.5,'DisplayName','DNN');
plot3(wp_DNN(:,1),wp_DNN(:,2),wp_DNN(:,3),'color','k','lineWidth',1.5,'DisplayName','actual')

e_dnn = sqrt(sum((wp_DNN(:,1:3)-[T_DNN.x_end_avg.*1000,T_DNN.y_end_avg.*1000,T_DNN.z_end_avg.*1000]).^2,2));


xlabel('x (mm)')
ylabel('y (mm)')
zlabel('z (mm)')

legend()

%% Plotting out trajectory runs (triangle)
figure(3); clf; hold on; grid on; axis equal;

% DNN
exp_DNN = 'line_2024_02_22_17_32_07';
motor_inputs = load(['./training/experiments/',exp_DNN,'/DNN_line_trajectory_inputs.mat']).output;
wp_DNN = load(['./training/experiments/',exp_DNN,'/line_trajectory.mat']).wp;
T_DNN = readtable(['./training/experiments/',exp_DNN,'/positions.csv']);

plot3(T_DNN.x_end_avg.*1000,T_DNN.y_end_avg.*1000,T_DNN.z_end_avg.*1000,'-^','LineWidth',1.5,'DisplayName','DNN');
plot3(wp_DNN(:,1),wp_DNN(:,2),wp_DNN(:,3),'color','k','lineWidth',1.5,'DisplayName','actual')

e_dnn = sqrt(sum((wp_DNN(:,1:3)-[T_DNN.x_end_avg.*1000,T_DNN.y_end_avg.*1000,T_DNN.z_end_avg.*1000]).^2,2));

xlabel('x (mm)')
ylabel('y (mm)')
zlabel('z (mm)')

legend()

figure(4)
hist(e_dnn)