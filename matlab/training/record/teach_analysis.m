close all; clear all; clc;
addpath('./util')

close all; clear all; clc;
warning('off', 'all');

home_pos = load('../state/home_measured.mat').pos;

lw = 2;
ms = 8;

%% Plotting out trajectory runs

record_path = './path_2024_08_09_15_37_37';
motor_inputs = load([record_path,'/DNN_trajectory_inputs.mat']).output;
wp = load([record_path,'/trajectory.mat']).wp;
T = readtable([record_path,'/data/positions.csv']);
T.x_end_avg = 1000.*(T.x_end_avg);
T.y_end_avg = 1000.*(T.y_end_avg);
T.z_end_avg = 1000.*(T.z_end_avg);

v_names = T.Properties.VariableNames(3:end);
T_home = array2table([home_pos(1:3),home_pos(5:7),home_pos(4)],'VariableNames',v_names);

% T_DNN = batch_transform(T_home,T_DNN);
T.x_end_avg = T.x_end_avg - T.x_end_avg(1);
T.y_end_avg = T.y_end_avg - T.y_end_avg(1);
T.z_end_avg = T.z_end_avg - T.z_end_avg(1);


%%

figure(1); clf;
hold on; grid on; axis equal;
set(gcf, 'Color', 'white');
set(gcf,'Units', 'inches','Position',[1,1,3.5,3]); % Adjusted for 2x1 layout

% DNN plot for the first tile
plot(-(T.y_end_avg),T.x_end_avg,'-','MarkerSize',ms,'LineWidth',lw,'DisplayName','DNN');
plot(-(wp(:,2)-home_pos(2)),wp(:,1)-home_pos(1),'-.','Color','k','LineWidth',lw,'DisplayName','actual');

% Adjusting the first plot's appearance
ax = gca;
ax.FontSize = 14;
ax.LineWidth = 1.5;


% Tile 2: X-Z
figure(2); clf;
hold on; grid on; axis equal;
set(gcf, 'Color', 'white');
set(gcf,'Units', 'inches','Position',[1,1,3.25,3]); % Adjusted for 2x1 layout


% Replotting for the second tile with X-Z view
% Note: Y-axis values are not needed for X-Z plot, so we use Z values as Y-axis here
plot(-(T.y_end_avg),T.z_end_avg,'-','MarkerSize',ms,'LineWidth',lw,'DisplayName','DNN');
plot(-(wp(:,2)-home_pos(2)),wp(:,3)-home_pos(3),'-.','Color','k','LineWidth',lw,'DisplayName','actual');
% legend('Location', 'bestoutside');

% Adjusting the second plot's appearance
ax = gca;
ax.FontSize = 14;
ax.LineWidth = 1.5;

%%
e = sqrt((T.x_end_avg-(wp(:,1)-home_pos(1))).^2 + (T.y_end_avg-(wp(:,2)-home_pos(2))).^2 + (T.z_end_avg-(wp(:,3)-home_pos(3))).^2);

figure(3);
hist(e);
mean(e)
