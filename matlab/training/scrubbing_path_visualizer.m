close all; clc; clear all;

wp = load('./inference/scrubbing/grill_wp.mat').wp;
data = readtable('../experiments/scrub_motive/Take 2025-04-01 03.36.33 PM.csv');
start_idx = find(data.Var1>1);
data = table2array(data(start_idx:end,3:10));
%% Plot

figure(1); clf; hold on; axis equal;

plot3(data(:,5),data(:,6),data(:,7));

plot3(wp(:,1),wp(:,2),wp(:,3))