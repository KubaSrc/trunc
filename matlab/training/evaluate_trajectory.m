close all; clear all; clc;
addpath('./util')

close all; clear all; clc;
warning('off', 'all');

% home_pos = load('./state/home_measured.mat').pos;

%% Test cases

evaluate('./inference/old_model_circle_trajectory_IDEAL.mat','./experiments/2025_01_24_00_54_30/positions.csv','old')
title("Old Model (Payload 655g)")

evaluate('./inference/655g_circle_trajectory.mat','./experiments/2025_01_23_23_34_01/positions.csv','new')
title("New Model (Payload 655g)")

evaluate('./inference/655g_circle_zeroed_trajectory.mat','./experiments/2025_01_24_01_14_54/positions.csv','new')
title("New Model no weight signal (Payload 655g)")

% evaluate('./inference/655g_circle_zeroed_trajectory.mat','./experiments/2025_01_24_01_14_54/positions.csv','new')
% title("New Model no weight signal (Payload 655g)")
% 
% evaluate('./inference/655g_circle_zeroed_trajectory.mat','./experiments/2025_01_24_01_14_54/positions.csv','new')
% title("New Model no weight signal (Payload 655g)")


%% Helper function

function[outputArg1,outputArg2] =  evaluate(ideal_trajectory,recorded_trajectory,model_type)
    
    T_ideal = load(ideal_trajectory);
    T_ideal = T_ideal.wp;
    T_model = readtable(recorded_trajectory);
     
    if model_type == 'new'
        % Ideal points
        T_x = (T_ideal(:,1)-T_ideal(1,1))./1000;
        T_y = (T_ideal(:,2)-T_ideal(1,2))./1000;
        T_z = (T_ideal(:,3)-T_ideal(1,3))./1000;
    else
        T_x = (T_ideal(:,1)-T_ideal(1,1))./1000;
        T_y = (T_ideal(:,3)-T_ideal(1,3))./1000;
        T_z = -((T_ideal(:,2)-T_ideal(1,2)))./1000;

    end

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