close all; clear all; clc;
addpath('./util')

home_pos = load('./state/home_measured.mat').pos;

% Calibration reference
exp_DNN = 'circle_2024_08_01_16_41_50';
wp_DNN = load(['./experiments/',exp_DNN,'/circle_trajectory.mat']).wp;
T_DNN = readtable(['./experiments/',exp_DNN,'/positions.csv']);

v_names = T_DNN.Properties.VariableNames(3:end);
T_home = array2table(home_pos,'VariableNames',v_names);

% Unit conversions
T_DNN.x_end_avg = 1000.*(T_DNN.x_end_avg);
T_DNN.y_end_avg = 1000.*(T_DNN.y_end_avg);
T_DNN.z_end_avg = 1000.*(T_DNN.z_end_avg);

dX = home_pos(1:3) - [T_DNN.x_end_avg(1),T_DNN.x_end_avg(2),T_DNN.x_end_avg(3)];

% Normalization
wp_DNN(:,1) = wp_DNN(:,1) - wp_DNN(1,1);
wp_DNN(:,2) = wp_DNN(:,2) - wp_DNN(1,2);
wp_DNN(:,3) = wp_DNN(:,3) - wp_DNN(1,3);


figure(1); clf; hold on; axis equal
plot3(T_DNN.x_end_avg,T_DNN.y_end_avg,T_DNN.z_end_avg)
plot3(wp_DNN(:,1),wp_DNN(:,2),wp_DNN(:,3))


%% Minimize the scaling difference
s_lower = 1;
s_upper = 2;

% Create an anonymous function that captures a and b
anonymous_obj_fun = @(s) obj_fun(s, T_DNN, wp_DNN);

s = fminbnd(anonymous_obj_fun,s_lower,s_upper);

function e = obj_fun(s,T_DNN,wp_DNN)
    e = sum(sqrt((s.*T_DNN.x_end_avg-wp_DNN(:,1)).^2 + (s.*T_DNN.y_end_avg-wp_DNN(:,2)).^2 + (T_DNN.z_end_avg-wp_DNN(:,3)).^2));
end

