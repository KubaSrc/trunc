close all; clear all; clc;
addpath('./util')

map = brewermap(9,'Set1');

% Masses associated with each data set
m0 = 400;
mass = m0+[50,100,127,255,578,255+578,578+564];
root_path = "./data/";
data_path = ["2024_02_19_21_08_57","-50g/2024_11_19_18_41_02","0g/2024_11_13_13_06_39","250g/2024_11_06_17_21_07","500gA/2024_11_07_15_54_15","750gA/2024_11_08_18_48_50","1000gAB\2024_11_09_16_32_42"];
file_path = "/positions.csv";

colors = lines(7);

% Pull in original data
T_full = readtable(root_path+data_path(1)+"/positions_norm_full.csv");
% Re-zero the data
T_full(:,4:6) = T_full(:,4:6) - T_full(1,4:6);
T_full_sub = T_full(:,4:10); 

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
writetable(T_csv_original_data, 'data/original_training_data.csv');

 % Add point cloud
 figure(1); clf; hold on; grid on;

 % Define legend labels
 legendLabels = ["450", "500", "527", "655", "978", "1233", "1542"];


 T_full_subA = table2array(T_full_sub)*1000;
 T_full_subA(:,8) = mass(1);
 s = scatter3(T_full_subA(:,1),T_full_subA(:,2),T_full_subA(:,3),'filled', 'MarkerFaceColor', colors(1,:), 'MarkerEdgeColor','k','LineWidth',1);
 s.SizeData=5;

% Pull in recent data
for trial = 2:length(mass)
    T_i = readtable(root_path+data_path(trial)+file_path);
    
    T_i(:,4:6) = T_i(:,4:6) - T_i(1,4:6);
    T_i_sub = T_i(:,4:10);
    T_i_sub = weighted_transform(T_i_sub);
    T_i_sub = batch_transform(T_full_sub,T_i_sub);
    
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
    s = scatter3(T_i_sub(:,1),T_i_sub(:,2),T_i_sub(:,3),'filled', 'MarkerFaceColor', colors(trial,:), 'MarkerEdgeColor','k','LineWidth',1);
    s.SizeData=30;
    
    disp(['Total points: ', num2str(size(T_full_subA, 1))]);
    
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