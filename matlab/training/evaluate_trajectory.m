close all; clear all; clc;
addpath('./util')

close all; clear all; clc;
warning('off', 'all');

home_pos = load('./state/home_measured.mat').pos;

%% Test cases

evaluate('./inference/no_weight_circle_trajectory.mat','./experiments/circle_2025_01_23_20_37_01/positions.csv')
title("Old Model (Payload 655g)")



%% Helper function

function[outputArg1,outputArg2] =  evaluate(ideal_trajectory,recorded_trajectory)
    
    T_ideal = load(ideal_trajectory);
    T_ideal = T_ideal.wp;
    T_model = readtable(recorded_trajectory);
     
    % Ideal points
    T_x = (T_ideal(:,1)-T_ideal(1,1))./1000;
    T_y = (T_ideal(:,3)-T_ideal(1,3))./1000;
    T_z = -(T_ideal(:,2)-T_ideal(1,2))./1000;
    
    % Model points
    T_m_x = T_model.x_end_avg-T_model.x_end_avg(1);
    T_m_y = T_model.y_end_avg-T_model.y_end_avg(1);
    T_m_z = T_model.z_end_avg-T_model.z_end_avg(1);
    
    figure(); clf; hold on;
    grid on; box on; axis equal
    
    plot3(T_m_x,T_m_y,T_m_z);
    plot3(T_x, T_y,T_z);
    
    e = mean(sqrt((T_x - T_m_x).^2 + (T_y - T_m_y).^2 + (T_z - T_m_z).^2));
    disp(1000.*e)
    
    xlabel("x");
    ylabel("y");
    zlabel("z");
end