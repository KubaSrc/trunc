close all; clear all; clc;

%%
figure(1); hold on; grid on;

load('./experiments/circle_2024_02_06_16_35_44/circle_trajectory.mat');
T = readtable('./experiments/circle_2024_02_06_16_35_44/positions.csv');

plot3(T.x_end_avg,)