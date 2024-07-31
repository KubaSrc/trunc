close all; clear all; clc;
addpath('./util')

close all; clear all; clc;
warning('off', 'all');

home_pos = load('./state/home_measured.mat').pos;

lw = 2;
ms = 8;

%% Plotting out trajectory runs (circle)

% DNN
exp_DNN = 'circle_2024_07_31_10_13_38';
motor_inputs = load(['./experiments/',exp_DNN,'/DNN_circle_trajectory_inputs.mat']).output;
wp_DNN = load(['./experiments/',exp_DNN,'/circle_trajectory.mat']).wp;
T_DNN = readtable(['./experiments/',exp_DNN,'/positions.csv']);

v_names = T_DNN.Properties.VariableNames(3:end);
T_home = array2table(home_pos,'VariableNames',v_names);

T_DNN = batch_transform(T_home,T_DNN);


figure(1); clf;
hold on; grid on; axis equal;
set(gcf, 'Color', 'white');
set(gcf,'Units', 'inches','Position',[1,1,3.5,3]); % Adjusted for 2x1 layout

% View 1: XY
xlim([-100,100]);
ylim([-100,100]);

% DNN plot for the first tile
plot(-(T_DNN.y_end_avg.*1000-home_pos(2)),T_DNN.x_end_avg.*1000-home_pos(1),'-','MarkerSize',ms,'LineWidth',lw,'DisplayName','DNN');
plot(-(wp_DNN(:,2)-home_pos(2)),wp_DNN(:,1)-home_pos(1),'-.','Color','k','LineWidth',lw,'DisplayName','actual');

% Adjusting the first plot's appearance
ax = gca;
ax.FontSize = 14;
ax.LineWidth = 1.5;


% Tile 2: X-Z
figure(2); clf;
hold on; grid on; axis equal;
set(gcf, 'Color', 'white');
set(gcf,'Units', 'inches','Position',[1,1,3.25,3]); % Adjusted for 2x1 layout

xlim([-100,100]);
ylim([0,100]);

% Replotting for the second tile with X-Z view
% Note: Y-axis values are not needed for X-Z plot, so we use Z values as Y-axis here
plot(-(T_DNN.y_end_avg.*1000-home_pos(2)),T_DNN.z_end_avg.*1000-home_pos(3),'-','MarkerSize',ms,'LineWidth',lw,'DisplayName','DNN');
plot(-(wp_DNN(:,2)-home_pos(2)),wp_DNN(:,3)-home_pos(3),'-.','Color','k','LineWidth',lw,'DisplayName','actual');
% legend('Location', 'bestoutside');

% Adjusting the second plot's appearance
ax = gca;
ax.FontSize = 14;
ax.LineWidth = 1.5;