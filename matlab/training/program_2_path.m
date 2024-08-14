close all; clc; clear all;

home_pos = load('./state/home_measured.mat').pos;
point_density = load('./state/point_density.mat').dist_per_point;

%% Convert motion to trajectory

export_traj = true;
pause_length = 1; % Pause before motor actuation
n_points = 100;

record_path = './record/path_2024_08_09_15_37_37';

wp_T = readtable([record_path,'/positions.csv']);
wp = table2array([wp_T(:,3:5),wp_T(:,9),wp_T(:,6:8)]); % Change to qw qx qy qz
wp = interp_waypoints(wp,n_points,"linear");

% Change of coordinate frame
wp(:,1:3) = 1000.*(wp(:,1:3)-wp(1,1:3))+home_pos(1:3);

figure(1); clf; hold on; grid on; axis equal

plot3(wp(:,1), wp(:,2), wp(:,3), '-', 'LineWidth', 1.5); % 'k:' makes the line black and dotted, 'LineWidth' sets the thickness
% scatter3(p_mat(:,1),p_mat(:,2),p_mat(:,3),'x','red','SizeData',100,'LineWidth',2)

xlabel('X-axis'); % Label for the x-axis
ylabel('Y-axis'); % Label for the y-axis
zlabel('Z-axis'); % Label for the z-axis

title('3D Line Plot with Grid'); % Title for the plot

if export_traj
    save([record_path,'/trajectory.mat'],'wp')
end

view([0,90])

%% Helper functions

function interpolatedWaypoints = interp_waypoints(waypoints, totalPoints, mode)
    % Extract positions and quaternions
    positions = waypoints(:, 1:3);
    quaternions = waypoints(:, 4:7);

    % Total number of waypoints
    numWaypoints = size(waypoints, 1);

    % Calculate cumulative distances
    cumulativeDistances = zeros(numWaypoints, 1);
    for i = 2:numWaypoints
        cumulativeDistances(i) = cumulativeDistances(i-1) + norm(positions(i, :) - positions(i-1, :));
    end

    % Normalize cumulative distances to [0, 1] for interpolation
    normalizedCumulativeDistances = cumulativeDistances / cumulativeDistances(end);

    % Interpolate positions using cubic spline
    tPositions = linspace(0, 1, totalPoints);
    if mode == "cubic"
        interpolatedPositions = spline(normalizedCumulativeDistances, positions', tPositions)';

    elseif mode == "linear"
    interpolatedPositions = interp1(normalizedCumulativeDistances, positions, tPositions, 'linear');
    end

    % Initialize quaternion interpolation
    interpolatedQuaternions = zeros(totalPoints, 4);

    % Interpolate quaternions
    for i = 1:totalPoints
        % Find the two original waypoints surrounding the current interpolated point
        t = tPositions(i);
        idx = find(normalizedCumulativeDistances <= t, 1, 'last');
        if idx == numWaypoints
            idx = numWaypoints - 1;
        end
        nextIdx = idx + 1;

        % Calculate the fraction between the two waypoints for SLERP
        tFraction = (t - normalizedCumulativeDistances(idx)) / (normalizedCumulativeDistances(nextIdx) - normalizedCumulativeDistances(idx));

        % Perform SLERP
        qStart = quaternions(idx, :) / norm(quaternions(idx, :));
        qEnd = quaternions(nextIdx, :) / norm(quaternions(nextIdx, :));
        interpolatedQuaternions(i, :) = quatinterp(qStart, qEnd, tFraction, 'slerp');
    end

    % Combine interpolated positions and quaternions
    interpolatedWaypoints = [interpolatedPositions, interpolatedQuaternions(:,[2,3,4,1])];
end

function plot_triad(x, y, z, q)
    figure(); clf; hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    arrowLength = 20; % Adjust the arrow length as needed
    headSize = 20;
        

    for i = 1:length(x)
        % Convert quaternion to rotation matrix
        R = quat2rotm(q(i,:));
        
        % Origin for the triad
        origin = [x(i), y(i), z(i)];
        
        % Directions for the triad arrows, transformed by R
        xDir = R(:,1)';
        yDir = R(:,2)';
        zDir = R(:,3)';
        
        % Plot arrows
        quiver3(origin(1), origin(2), origin(3), xDir(1), xDir(2), xDir(3), arrowLength, 'r', 'LineWidth', 2, 'MaxHeadSize', headSize);
        quiver3(origin(1), origin(2), origin(3), yDir(1), yDir(2), yDir(3), arrowLength, 'g', 'LineWidth', 2, 'MaxHeadSize', headSize);
        quiver3(origin(1), origin(2), origin(3), zDir(1), zDir(2), zDir(3), arrowLength, 'b', 'LineWidth', 2, 'MaxHeadSize', headSize);
    end
    
    hold off;
end