close all; clear all; clc;
warning('off', 'all');

%%
figure(1); clf; hold on; grid on;

exp = 'circle_2024_02_15_22_07_28';

motor_inputs = load(['./training/experiments/',exp,'/LSTM_circle_trajectory_inputs.mat']).output;
wp = load(['./training/experiments/',exp,'/circle_trajectory.mat']).wp;
T = readtable(['./training/experiments/',exp,'/positions.csv']);

plot3(T.x_end_avg.*1000,T.y_end_avg.*1000,T.z_end_avg.*1000,'-o','LineWidth',1.5,'DisplayName','measured');
plot3(wp(:,1),wp(:,2),wp(:,3),'-x','lineWidth',1.5,'DisplayName','actual')

xlabel('x (mm)')
ylabel('y (mm)')
zlabel('z (mm)')

legend()


l_delta = load("./training/trajectory/delta_fast_repeat.mat").delta_fast;