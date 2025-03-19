close all; clear all; clc;
warning('off', 'all');

home_pos = load('./training/state/home_triad_measured.mat').pos;
old_home = load('./training/state/home_measured.mat').pos;

map = brewermap(9,'Set1');

export_fig = true;

lw = 2;
ms = 8;

%% Plotting out trajectory runs (circle)

% DNN Feb4 Model
% In the order of:  Old Model           New Algorithm Model   
list_exp_DNN = {'2025_01_24_00_54_30', '_2025_02_04_20_16_34', };
list_motor_inputs = {'/old_model_circle_trajectory_inputs','/feb4_655g_inputs.mat'};
list_wp_DNN = {'/old_model_circle_trajectory', '/655g_circle_trajectory.mat'};
for i = 1:length(list_exp_DNN)
    exp_DNN = list_exp_DNN{i};

    motor_inputs = load(['./training/experiments/', exp_DNN, list_motor_inputs{i}]).output;
    wp_DNN = load(['./training/experiments/',exp_DNN, list_wp_DNN{i}]).wp;
    T_DNN = readtable(['./training/experiments/',exp_DNN,'/positions.csv']);

    %transform the table for anything other than the old dataset
    T_DNN = batch_quaternion_transform(T_DNN);
   
    % figure(1); clf;
    % hold on; grid on; axis equal;
    % set(gcf, 'Color', 'white');
    % set(gcf,'Units', 'inches','Position',[1,1,3.5,3]); % Adjusted for 2x1 layout

    % Create new figure for each iteration for XY plot
    fig_xy = figure;  % Creates a new figure
    clf(fig_xy);  % Clears the figure window
    hold on; grid on; axis equal; box on;
    set(fig_xy, 'Color', 'white');
    set(fig_xy, 'Units', 'inches', 'Position', [1, 1, 3.5, 3]);  % Adjusted for 2x1 layout
    
    % View 1: XY
    xlim([-75,75]);
    ylim([-20,100]);
    xticks([-50,0,50])
    yticks(-20:20:100)
    
    % DNN plot for the first tile
    if i == 1 
        plot(-(T_DNN.x_end_avg.*1000)-home_pos(1),(T_DNN.y_end_avg.*1000)-home_pos(2),'-','MarkerSize',ms,'LineWidth',lw,'DisplayName','DNN');
        plot(wp_DNN(:,2)-old_home(2),wp_DNN(:,3)-old_home(3),'-.','Color','k','LineWidth',lw,'DisplayName','actual'); 
         % Mark start and end points
        scatter(-(T_DNN.x_end_avg(1).*1000)-home_pos(1), (T_DNN.y_end_avg(1).*1000)-home_pos(2), ...
            80, 'o', 'filled', 'CData', map(3,:)); % Start point
        scatter(-(T_DNN.x_end_avg(end).*1000)-home_pos(1), (T_DNN.y_end_avg(end).*1000)-home_pos(2), ...
            100, 's', 'filled', 'CData', map(1,:)); % End point
    else
        plot(-(T_DNN.x_end_avg-T_DNN.x_end_avg(1)).*1000,(T_DNN.y_end_avg-T_DNN.y_end_avg(1)).*1000,'-','MarkerSize',ms,'LineWidth',lw,'DisplayName','DNN');
        plot(-(wp_DNN(:,1)-home_pos(1)),wp_DNN(:,2)-home_pos(2),'-.','Color','k','LineWidth',lw,'DisplayName','actual');
         % Mark start and end points
        scatter(-(T_DNN.x_end_avg(1) - T_DNN.x_end_avg(1)).*1000, (T_DNN.y_end_avg(1) - T_DNN.y_end_avg(1)).*1000, ...
            80, 'o', 'filled', 'CData', map(3,:)); % Start point
        scatter(-(T_DNN.x_end_avg(end) - T_DNN.x_end_avg(1)).*1000, (T_DNN.y_end_avg(end) - T_DNN.y_end_avg(1)).*1000, ...
            100, 's', 'filled', 'CData', map(1,:)); % End point
    end

    % Adjusting the first plot's appearance
    ax = gca;
    ax.FontSize = 14;
    ax.LineWidth = 1.5;    
    
    if export_fig
        exportgraphics(fig_xy, ['../figures/trajectory/circle-x-y-', num2str(i), '.emf'], 'ContentType', 'vector');
    end
    

    % Create new figure for X-Z plot
    fig_xz = figure;  % Creates a new figure for X-Z plot
    clf(fig_xz);  % Clears the figure window
    hold on; grid on; axis equal; box on;
    set(fig_xz, 'Color', 'white');
    set(fig_xz, 'Units', 'inches', 'Position', [1, 1, 3.25, 3]);  % Adjusted for 2x1 layout
    

    xlim([-75,75]);
    ylim([-75,75]);
    
    % Replotting for the second tile with X-Z view
    % Note: Y-axis values are not needed for X-Z plot, so we use Z values as Y-axis here


    if i == 1 
        plot(-(T_DNN.x_end_avg-T_DNN.x_end_avg(1)).*1000,-(T_DNN.z_end_avg-T_DNN.z_end_avg(1)).*1000,'-','MarkerSize',ms,'LineWidth',lw,'DisplayName','DNN');
        plot(-(wp_DNN(:,1)-old_home(1)),wp_DNN(:,2)-old_home(2),'-.','Color','k','LineWidth',lw,'DisplayName','actual'); 
        % Mark start and end points
        scatter(-(T_DNN.x_end_avg(1) - T_DNN.x_end_avg(1)).*1000, -(T_DNN.z_end_avg(1) - T_DNN.z_end_avg(1)).*1000, ...
            80, 'o', 'filled', 'CData', map(3,:)); % Start point
        scatter(-(T_DNN.x_end_avg(end) - T_DNN.x_end_avg(1)).*1000, -(T_DNN.z_end_avg(end) - T_DNN.z_end_avg(1)).*1000, ...
            100, 's', 'filled', 'CData', map(1,:)); % End point

    else
        plot(-(T_DNN.x_end_avg - T_DNN.x_end_avg(1)).*1000,(T_DNN.z_end_avg - T_DNN.z_end_avg(1)).*1000,'-','MarkerSize',ms,'LineWidth',lw,'DisplayName','DNN');
        plot(-(wp_DNN(:,1)-home_pos(1)),wp_DNN(:,3)-home_pos(3),'-.','Color','k','LineWidth',lw,'DisplayName','actual');
        % Mark start and end points
        scatter(-(T_DNN.x_end_avg(1) - T_DNN.x_end_avg(1)).*1000, (T_DNN.z_end_avg(1) - T_DNN.z_end_avg(1)).*1000, ...
            80, 'o', 'filled', 'CData', map(3,:)); % Start point
        scatter(-(T_DNN.x_end_avg(end) - T_DNN.x_end_avg(1)).*1000, (T_DNN.z_end_avg(end) - T_DNN.z_end_avg(1)).*1000, ...
            100, 's', 'filled', 'CData', map(1,:)); % End point

    end

    % legend('Location', 'bestoutside');
    
    % Adjusting the second plot's appearance
    ax = gca;
    ax.FontSize = 14;
    ax.LineWidth = 1.5;
    
    % Export the figure as a vector graphic
    if export_fig
        exportgraphics(fig_xz, ['../figures/trajectory/circle-y-z-', num2str(i), '.emf'], 'ContentType', 'vector');
    end

    % Create new figure for position vs time plot
    circle_seconds = linspace(0,500,100);

    x = -(T_DNN.y_end_avg.*1000); % home 2
    y = T_DNN.x_end_avg.*1000; % home 1
    z = T_DNN.z_end_avg.*1000; % home 3
    
    x_ref = -(wp_DNN(:,2));
    y_ref = wp_DNN(:,1);
    z_ref = wp_DNN(:,3);

    fig_xz = figure;  % Creates a new figure for pos vs time plot
    clf(fig_xz);  % Clears the figure window
    hold on; grid on; box on;
    set(fig_xz, 'Color', 'white');
    set(fig_xz, 'Units', 'inches', 'Position', [1, 1, 3.25, 3]);  % Adjusted for 2x1 layout
    
    xlim([0,500]);
    ylim([-75,100]);


    if i == 1 
        plot(circle_seconds,-(x+home_pos(2)),'-','Color',map(4,:),'MarkerSize',ms,'LineWidth',lw,'DisplayName','z Actual'); %z v time
        plot(circle_seconds,z_ref - old_home(3),'-.','Color',map(4,:),'MarkerSize',ms,'LineWidth',lw,'DisplayName','z Ideal');
        plot(circle_seconds,y-home_pos(1),'-','Color',map(5,:),'MarkerSize',ms,'LineWidth',lw,'DisplayName','y Actual'); %y v time
        plot(circle_seconds,y_ref-old_home(1),'-.','Color',map(5,:),'MarkerSize',ms,'LineWidth',lw,'DisplayName','y Ideal'); 
        plot(circle_seconds,z-home_pos(3),'-','MarkerSize',ms,'LineWidth',lw,'DisplayName','x Actual'); %x v time
        plot(circle_seconds,x_ref-old_home(2),'-.','Color',map(3,:),'MarkerSize',ms,'LineWidth',lw,'DisplayName','x Ideal');        
    else
        plot(circle_seconds, -(x - x(1)),'-','Color',map(4,:),'MarkerSize',ms,'LineWidth',lw,'DisplayName','x Actual');
        plot(circle_seconds, -(x_ref-x_ref(1)),'-.','Color',map(4,:),'LineWidth',lw,'DisplayName','x Ideal');
        plot(circle_seconds, y - y(1),'-','Color',map(5,:),'MarkerSize',ms,'LineWidth',lw,'DisplayName','y Actual');
        plot(circle_seconds,y_ref-y_ref(1),'-.','Color',map(5,:),'LineWidth',lw,'DisplayName','y Ideal');
        plot(circle_seconds, z - z(1),'-','Color',map(3,:),'MarkerSize',ms,'LineWidth',lw,'DisplayName','z Actual');
        plot(circle_seconds,z_ref-z_ref(1),'-.','Color',map(3,:),'LineWidth',lw,'DisplayName','z Ideal');
    end    

    ax = gca;
    ax.FontSize = 14;
    ax.LineWidth = 1.5;   


    % Export the figure as a vector graphic
    if export_fig
        exportgraphics(fig_xz, ['../figures/trajectory/pos-', num2str(i), '.emf'], 'ContentType', 'vector');
    end


    % Plot out orientation error

    ref_quat = [wp_DNN(:,7),wp_DNN(:,4),wp_DNN(:,5),wp_DNN(:,6)];
    measured_quat = [T_DNN.qw_end_avg,T_DNN.qx_end_avg,T_DNN.qy_end_avg,T_DNN.qz_end_avg];
    measured_quat = remove_twist(measured_quat);
    theta = 2 * acos(abs(dot(ref_quat, measured_quat,2)));
    circle_orientation_error = rad2deg(theta);
    
    fig_quat = figure; 
    clf(fig_quat);
    hold on; grid on; box on;
    set(gcf, 'Color', 'white');
    set(gcf,'Units', 'inches','Position',[1, 1, 3.5, 3]); % Adjusted for 2x1 layout
    
    plot(circle_seconds,circle_orientation_error,'color','k','LineWidth',lw)
    
    ax = gca;
    ax.FontSize = 14;
    ax.LineWidth = 1.5;
    ylim([0,4])
    xlim([0,500])
    xticks([0:100:500])
    
    % Export the figure as a vector graphic
    if export_fig
        exportgraphics(fig_quat, ['../figures/trajectory/circle-angle-error-', num2str(i), '.emf'], 'ContentType', 'vector');
    end

end


%% Functions

function secondsArray = find_seconds(dateStrings)
    % Convert string to datetime
    dateTimes = datetime(dateStrings, 'InputFormat', 'dd-MMM-yyyy HH:mm:ss');
    % Initialize an array to store seconds
    secondsArray = zeros(size(dateTimes, 1), 1);
    % Loop through each datetime and calculate seconds
    for i = 1:length(dateTimes)
        % Extract hours, minutes, and seconds
        [hours, minutes, seconds] = hms(dateTimes(i));
        % Calculate total seconds
        totalSeconds = hours * 3600 + minutes * 60 + seconds;
        % Store in the array
        secondsArray(i) = totalSeconds;
    end
    secondsArray = secondsArray - secondsArray(1);
end

function [Q_new] = remove_twist(Q)

    Q_new = zeros(size(Q));
    q_1 = Q(1,:);
    
    eul = quat2eul(q_1, 'XYZ');
    theta = -eul(3);
    Rz = [cos(theta) -sin(theta) 0;
          sin(theta) cos(theta)  0;
          0          0           1];
    
    for i = 1:size(Q)
        q_i = Q(i,:);
        R_i = quat2rotm(q_i);
        R_new = R_i*Rz;
        q_new = rotm2quat(R_new);
        Q_new(i,:) = q_new;
    end
end

