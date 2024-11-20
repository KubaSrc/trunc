close all; clear all; clc;
addpath('./util')

T_home = readtable('./data/2024_02_19_21_08_57/positions.csv');
T_home = T_home(1,4:10);

% Calibration reference
exp_DNN = 'circle_2024_11_12_21_46_36';
wp_DNN = load(['./experiments/',exp_DNN,'/circle_trajectory.mat']).wp;
T_DNN = readtable(['./experiments/',exp_DNN,'/positions.csv']);
T_DNN = T_DNN(:,3:9);
T_DNN = weighted_transform(T_DNN);

% Calibration reference
exp_DNN_aug = 'circle_2024_08_05_10_29_17';
wp_DNN_aug = load(['./experiments/',exp_DNN,'/circle_trajectory.mat']).wp;
T_DNN_aug = readtable(['./experiments/',exp_DNN,'/positions.csv']);
T_DNN_aug = T_DNN_aug(:,3:9);
T_DNN_aug = weighted_transform(T_DNN_aug);

% Unit conversions for DNN
T_DNN.x_end_avg = 1000.*(T_DNN.x_end_avg);
T_DNN.y_end_avg = 1000.*(T_DNN.y_end_avg);
T_DNN.z_end_avg = 1000.*(T_DNN.z_end_avg);

% Unit conversions for DNN
T_DNN_aug.x_end_avg = 1000.*(T_DNN_aug.x_end_avg);
T_DNN_aug.y_end_avg = 1000.*(T_DNN_aug.y_end_avg);
T_DNN_aug.z_end_avg = 1000.*(T_DNN_aug.z_end_avg);

% Unit conversions for home pos
T_home.x_end_avg = 1000.*T_home.x_end_avg;
T_home.y_end_avg = 1000.*T_home.y_end_avg;
T_home.z_end_avg = 1000.*T_home.z_end_avg;

T_DNN_batch = batch_transform(T_home,T_DNN);

figure(); clf; hold on;

dX = T_home(1,1:3) - [T_DNN.x_end_avg(1),T_DNN.y_end_avg(2),T_DNN.z_end_avg(3)];

% Normalization
wp_DNN(:,1) = wp_DNN(:,1) - wp_DNN(1,1);
wp_DNN(:,2) = wp_DNN(:,2) - wp_DNN(1,2);
wp_DNN(:,3) = wp_DNN(:,3) - wp_DNN(1,3);

T_DNN_aug.x_end_avg = T_DNN_aug.x_end_avg-T_DNN_aug.x_end_avg(1);
T_DNN_aug.y_end_avg = T_DNN_aug.y_end_avg-T_DNN_aug.y_end_avg(1);
T_DNN_aug.z_end_avg = T_DNN_aug.z_end_avg-T_DNN_aug.z_end_avg(1);

T_DNN_batch.x_end_avg = T_DNN_batch.x_end_avg-T_DNN_batch.x_end_avg(1);
T_DNN_batch.y_end_avg = T_DNN_batch.y_end_avg-T_DNN_batch.y_end_avg(1);
T_DNN_batch.z_end_avg = T_DNN_batch.z_end_avg-T_DNN_batch.z_end_avg(1);

figure(1); clf; hold on; axis equal
plot3(T_DNN_aug.x_end_avg,T_DNN_aug.y_end_avg,T_DNN_aug.z_end_avg)
plot3(wp_DNN(:,1),wp_DNN(:,2),wp_DNN(:,3))
plot3(T_DNN_batch.x_end_avg,T_DNN_batch.y_end_avg,T_DNN_batch.z_end_avg)


%% FOR INFERENCE (scaling up the data)
s_lower = 0;
s_upper = 2;

% Create an anonymous function that captures a and b
anonymous_obj_fun = @(s) obj_fun(s, T_DNN, wp_DNN);

s = fminbnd(anonymous_obj_fun,s_lower,s_upper);

fileName = datestr(datetime('now'), '_yyyy_mm_dd_HH_MM_SS');

save(['./calibration/cal',fileName,'.mat'],"s","dX")

figure(2); clf; hold on; axis equal
plot3(s.*T_DNN.x_end_avg,s.*T_DNN.y_end_avg,s.*T_DNN.z_end_avg)
plot3(wp_DNN(:,1),wp_DNN(:,2),wp_DNN(:,3))

e = sqrt((s.*T_DNN.x_end_avg-wp_DNN(:,1)).^2 + (s.*T_DNN.y_end_avg-wp_DNN(:,2)).^2 + (s.*T_DNN.z_end_avg-wp_DNN(:,3)).^2);

figure(3);
hist(e);
mean(e)


%% FOR TRAINING (scaling down the reference)

s_lower = 0;
s_upper = 2;

% Create an anonymous function that captures a and b
anonymous_obj_fun = @(s) obj_fun_train(s, T_DNN_batch, wp_DNN);

s = fminbnd(anonymous_obj_fun,s_lower,s_upper);

fileName = datestr(datetime('now'), '_yyyy_mm_dd_HH_MM_SS');

save(['./calibration/train_cal',fileName,'.mat'],"s","dX")

figure(2); clf; hold on; axis equal
plot3(T_DNN_batch.x_end_avg,T_DNN_batch.y_end_avg,T_DNN_batch.z_end_avg)
plot3(s.*wp_DNN(:,1),s.*wp_DNN(:,2),s.*wp_DNN(:,3))

e = sqrt((T_DNN_batch.x_end_avg-s.*wp_DNN(:,1)).^2 + (T_DNN_batch.y_end_avg-s.*wp_DNN(:,2)).^2 + (T_DNN_batch.z_end_avg-s.*wp_DNN(:,3)).^2);

figure(3);
hist(e);
mean(e)

%%

function e = obj_fun(s,T,wp)
    e = sum(sqrt((s.*T.x_end_avg-wp(:,1)).^2 + (s.*T.y_end_avg-wp(:,2)).^2 + (s.*T.z_end_avg-wp(:,3)).^2));
end

function e = obj_fun_train(s,T,wp)
    e = sum(sqrt((T.x_end_avg-s.*wp(:,1)).^2 + (T.y_end_avg-s.*wp(:,2)).^2 + (T.z_end_avg-s.*wp(:,3)).^2));
end



