close all; clear all; clc;
addpath('./util')

T_old = readtable('./data/2024_02_19_21_08_57/positions.csv');
T_old = T_old(1:100,4:10);

s = load("./calibration/train_cal_2024_11_13_11_30_13.mat").s;
s=.9;
% Load in new data and apply transformations
T_new = readtable('./data/2024_11_13_10_48_26/positions.csv');
T_new = T_new(1:100,4:10);
T_new = weighted_transform(T_new);
T_new = batch_transform(T_old,T_new);

figure(1); clf; hold on; grid on

% Rescale data
T_old.x_end_avg = s.*1000.*(T_old.x_end_avg - T_old.x_end_avg(1));
T_old.y_end_avg = s.*1000.*(T_old.y_end_avg - T_old.y_end_avg(1));
T_old.z_end_avg = s.*1000.*(T_old.z_end_avg - T_old.z_end_avg(1));

T_new.x_end_avg = 1000.*(T_new.x_end_avg - T_new.x_end_avg(1));
T_new.y_end_avg = 1000.*(T_new.y_end_avg - T_new.y_end_avg(1));
T_new.z_end_avg = 1000.*(T_new.z_end_avg - T_new.z_end_avg(1));


plot3(T_old.x_end_avg,T_old.y_end_avg,T_old.z_end_avg,'r');
plot3(T_new.x_end_avg,T_new.y_end_avg,T_new.z_end_avg,'b');

e = sqrt((T_old.x_end_avg-T_new.x_end_avg).^2 + (T_old.y_end_avg-T_new.y_end_avg).^2 + (T_old.z_end_avg-T_new.z_end_avg).^2);
mean(e)

figure(2); 
hist(e)
