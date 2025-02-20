close all; clear all; clc;
warning('off', 'all');

home_pos = load('./state/home_triad_measured.mat').pos;
old_home = load('./state/home_measured.mat').pos;


export_fig = true;

lw = 2;
ms = 8;

%% Plotting out trajectory runs (circle)

% DNN Feb4 Model
% In the order of:  Old Model     Old Algorithm Model      New Algorithm Model   
list_exp_DNN = {'2025_01_24_00_54_30', '2025_01_23_23_34_01', '_2025_02_04_20_16_34', };
list_motor_inputs = {'/old_model_circle_trajectory_inputs', '/all_new_655g_inputs.mat','/feb4_655g_inputs.mat'};
list_wp_DNN = {'/old_model_circle_trajectory', '/655g_circle_trajectory.mat', '/655g_circle_trajectory.mat'};
for i = 1:length(list_exp_DNN)
    exp_DNN = list_exp_DNN{i};

    motor_inputs = load(['./experiments/', exp_DNN, list_motor_inputs{i}]).output;
    wp_DNN = load(['./experiments/',exp_DNN, list_wp_DNN{i}]).wp;
    T_DNN = readtable(['./experiments/',exp_DNN,'/positions.csv']);
   
    % figure(1); clf;
    % hold on; grid on; axis equal;
    % set(gcf, 'Color', 'white');
    % set(gcf,'Units', 'inches','Position',[1,1,3.5,3]); % Adjusted for 2x1 layout

    % Create new figure for each iteration for XY plot
    fig_xy = figure;  % Creates a new figure
    clf(fig_xy);  % Clears the figure window
    hold on; grid on; axis equal;
    set(fig_xy, 'Color', 'white');
    set(fig_xy, 'Units', 'inches', 'Position', [1, 1, 3.5, 3]);  % Adjusted for 2x1 layout
    
    % View 1: XY
    xlim([-100,100]);
    ylim([-100,100]);
    
    % DNN plot for the first tile
    if i == 1 
        plot(-(T_DNN.x_end_avg.*1000)-home_pos(1),(T_DNN.y_end_avg.*1000)-home_pos(2),'-','MarkerSize',ms,'LineWidth',lw,'DisplayName','DNN');
        plot(wp_DNN(:,2)-old_home(2),wp_DNN(:,3)-old_home(3),'-.','Color','k','LineWidth',lw,'DisplayName','actual'); 
    else
        plot(-(T_DNN.x_end_avg-T_DNN.x_end_avg(1)).*1000,(T_DNN.y_end_avg-T_DNN.y_end_avg(1)).*1000,'-','MarkerSize',ms,'LineWidth',lw,'DisplayName','DNN');
        plot(-(wp_DNN(:,1)-home_pos(1)),wp_DNN(:,2)-home_pos(2),'-.','Color','k','LineWidth',lw,'DisplayName','actual');
    end
    % Adjusting the first plot's appearance
    ax = gca;
    ax.FontSize = 14;
    ax.LineWidth = 1.5;    
    
    if export_fig
        exportgraphics(fig_xy, ['../../figures/trajectory/circle-x-y-', num2str(i), '.emf'], 'ContentType', 'vector');
    end
    
    % % Tile 2: X-Z
    % figure(2); clf;
    % hold on; grid on; axis equal;
    % set(gcf, 'Color', 'white');
    % set(gcf,'Units', 'inches','Position',[1,1,3.25,3]); % Adjusted for 2x1 layout

    % Create new figure for X-Z plot
    fig_xz = figure;  % Creates a new figure for X-Z plot
    clf(fig_xz);  % Clears the figure window
    hold on; grid on; axis equal;
    set(fig_xz, 'Color', 'white');
    set(fig_xz, 'Units', 'inches', 'Position', [1, 1, 3.25, 3]);  % Adjusted for 2x1 layout
    
    xlim([-100,100]);
    ylim([-100,100]);
    
    % Replotting for the second tile with X-Z view
    % Note: Y-axis values are not needed for X-Z plot, so we use Z values as Y-axis here

    if i == 1 
        plot(-(T_DNN.x_end_avg.*1000)-home_pos(1),(T_DNN.z_end_avg.*1000)-home_pos(3),'-','MarkerSize',ms,'LineWidth',lw,'DisplayName','DNN');
        plot(wp_DNN(:,2)-old_home(2),wp_DNN(:,1)-old_home(1),'-.','Color','k','LineWidth',lw,'DisplayName','actual'); 
    else
        plot(-(T_DNN.x_end_avg - T_DNN.x_end_avg(1)).*1000,(T_DNN.z_end_avg - T_DNN.z_end_avg(1)).*1000,'-','MarkerSize',ms,'LineWidth',lw,'DisplayName','DNN');
        plot(-(wp_DNN(:,1)-home_pos(1)),wp_DNN(:,3)-home_pos(3),'-.','Color','k','LineWidth',lw,'DisplayName','actual');
    end
    % legend('Location', 'bestoutside');
    
    % Adjusting the second plot's appearance
    ax = gca;
    ax.FontSize = 14;
    ax.LineWidth = 1.5;
    
    % Export the figure as a vector graphic
    if export_fig
        exportgraphics(fig_xz, ['../../figures/trajectory/circle-y-z-', num2str(i), '.emf'], 'ContentType', 'vector');
    end
end
%% Plotting out trajectory runs (triangle)

% DNN
exp_DNN = 'triangle_2024_02_22_15_48_39';
motor_inputs = load(['./training/experiments/',exp_DNN,'/DNN_triangle_trajectory_inputs.mat']).output;
wp_DNN = load(['./training/experiments/',exp_DNN,'/triangle_trajectory.mat']).wp;
T_DNN = readtable(['./training/experiments/',exp_DNN,'/positions.csv']);

figure(3); clf;
hold on; grid on; axis equal;
set(gcf, 'Color', 'white');
set(gcf,'Units', 'inches','Position',[1,1,3.5,3]); % Adjusted for 2x1 layout

% View 1: XY
xlim([-50,150])

ylim([-100,100])

T_DDN(:,:) = T_DNN - T_DNN(1,:);
wp_DDN(:,:) = wp_DNN - wp_DNN(1,:);

% DNN plot for the first tile
plot(-(T_DNN.y_end_avg.*1000-home_pos(2)),T_DNN.x_end_avg.*1000-home_pos(1),'-','MarkerSize',ms,'LineWidth',lw,'DisplayName','');
plot(-(wp_DNN(:,2)-home_pos(2)),wp_DNN(:,1)-home_pos(1),'-.','Color','k','LineWidth',lw,'DisplayName','');

% Adjusting the first plot's appearance
ax = gca;
ax.FontSize = 14;
ax.LineWidth = 1.5;

if export_fig
    exportgraphics(gcf,'../figures/trajectory/triangle-x-y.emf','ContentType', 'vector');
end

% Tile 2: X-Z
figure(4); clf;
hold on; grid on; axis equal;
set(gcf, 'Color', 'white');
set(gcf,'Units', 'inches','Position',[1,1,3.25,3]); % Adjusted for 2x1 layout

xlim([-50,150]);
ylim([0,100]);

% Replotting for the second tile with X-Z view
% Note: Y-axis values are not needed for X-Z plot, so we use Z values as Y-axis here
plot(-(T_DNN.y_end_avg.*1000-home_pos(2)),T_DNN.z_end_avg.*1000-home_pos(3),'-','MarkerSize',ms,'LineWidth',lw,'DisplayName','DNN');
plot(-(wp_DNN(:,2)-home_pos(2)),wp_DNN(:,3)-home_pos(3),'-.','Color','k','LineWidth',lw,'DisplayName','actual');

% Adjusting the second plot's appearance
ax = gca;
ax.FontSize = 14;
ax.LineWidth = 1.5;

if export_fig
    exportgraphics(gcf,'../figures/trajectory/triangle-y-z.emf','ContentType', 'vector');
end
%% Plotting out trajectory runs (line)

% DNN
exp_DNN = 'line_2024_02_22_17_32_07';
motor_inputs = load(['./training/experiments/',exp_DNN,'/DNN_line_trajectory_inputs.mat']).output;
wp_DNN = load(['./training/experiments/',exp_DNN,'/line_trajectory.mat']).wp;
T_DNN = readtable(['./training/experiments/',exp_DNN,'/positions.csv']);


figure(5); clf;
hold on; grid on; axis equal;
set(gcf, 'Color', 'white');
set(gcf,'Units', 'inches','Position',[1,1,3.5,3]); % Adjusted for 2x1 layout


% DNN plot for the first tile
plot(-(T_DNN.y_end_avg.*1000-home_pos(2)),T_DNN.x_end_avg.*1000-home_pos(1),'-','MarkerSize',ms,'LineWidth',lw,'DisplayName','DNN');
plot(-(wp_DNN(:,2)-home_pos(2)),wp_DNN(:,1)-home_pos(1),'-.','Color','k','LineWidth',lw,'DisplayName','actual');
xlim([-100,100])
ylim([-100,100])


% Adjusting the first plot's appearance
ax = gca;
ax.FontSize = 14;
ax.LineWidth = 1.5;

if export_fig
    exportgraphics(gcf,'../figures/trajectory/stairs-x-y.emf','ContentType', 'vector');
end


% Tile 2: X-Z
figure(6); clf;
hold on; grid on; axis equal;
set(gcf, 'Color', 'white');
set(gcf,'Units', 'inches','Position',[1,1,3.25,3]); % Adjusted for 2x1 layout

xlim([-100,100])
ylim([0,100])

% Replotting for the second tile with X-Z view
% Note: Y-axis values are not needed for X-Z plot, so we use Z values as Y-axis here
plot(-(T_DNN.y_end_avg.*1000-home_pos(2)),T_DNN.z_end_avg.*1000-home_pos(3),'-','MarkerSize',ms,'LineWidth',lw,'DisplayName','DNN');
plot(-(wp_DNN(:,2)-home_pos(2)),wp_DNN(:,3)-home_pos(3),'-.','Color','k','LineWidth',lw,'DisplayName','actual');
% legend('Location', 'bestoutside');

% Adjusting the second plot's appearance
ax = gca;
ax.FontSize = 14;
ax.LineWidth = 1.5;

if export_fig
    exportgraphics(gcf,'../figures/trajectory/stairs-y-z.emf','ContentType', 'vector');
end