close all; clc; clear all;

home_pos = load('./state/home_measured.mat').pos;
point_density = load('./state/point_density.mat').dist_per_point;

%% First trajectory (circle)

export_traj = true;

tool_rot = eye(3);
tool_quat = rotm2quat(tool_rot);

n = 20;
r = 80;
z_plane = 60;
theta = linspace(0,2*pi,n).';

x_c = r.*cos(theta);
y_c = r.*sin(theta);
z_c = repmat(z_plane,size(theta));

wp = [zeros(1,3), tool_quat;
      x_c, y_c, z_c, repmat(tool_quat,[n,1])];
wp = wp + home_pos;
wp = interp_waypoints(wp,100);

figure(1); clf; hold on; grid on

plot3(wp(:,1), wp(:,2), wp(:,3), 'x-', 'LineWidth', 1.5); % 'k:' makes the line black and dotted, 'LineWidth' sets the thickness

xlabel('X-axis'); % Label for the x-axis
ylabel('Y-axis'); % Label for the y-axis
zlabel('Z-axis'); % Label for the z-axis

title('3D Line Plot with Grid'); % Title for the plot

if export_traj
    save('./inference/circle_trajectory.mat','wp')
end

%% Second trajectory (steps)

% tool_rot = eye(3);
% tool_quat = rotm2quat(tool_rot);
% 
% l = 80;
% d = -40;
% z_plane = 60;
% 
% wp_xyz = [0,0,0; % Origin
%           l,0,z_plane; % #1 (start)
%           l,0,z_plane-d;
%           l,0,z_plane; % #1 (end)
%           -l*sin(pi/6),-l*cos(pi/6),z_plane; % #2 (start)
%           -l*sin(pi/6),-l*cos(pi/6),z_plane-d;
%           -l*sin(pi/6),-l*cos(pi/6),z_plane; % #2 (start)
%           l*sin(pi/6),-l*cos(pi/6),z_plane; % #3 (start)
%           l*sin(pi/6),-l*cos(pi/6),z_plane-d;
%           l*sin(pi/6),-l*cos(pi/6),z_plane; % #3 (end)
%           ];
% 
% n = length(wp_xyz);
% 
% wp = [wp_xyz, repmat(tool_quat,[n,1])];
% wp = wp + home_pos;
% wp = interp_waypoints(wp,200);
% 
% figure(2); clf; hold on; grid on
% 
% plot3(wp(:,1), wp(:,2), wp(:,3), 'x-', 'LineWidth', 1.5); % 'k:' makes the line black and dotted, 'LineWidth' sets the thickness
% 
% xlabel('X-axis'); % Label for the x-axis
% ylabel('Y-axis'); % Label for the y-axis
% zlabel('Z-axis'); % Label for the z-axis
% 
% title('3D Line Plot with Grid'); % Title for the plot
% 
% if export_traj
%     save('./inference/circle_trajectory.mat','wp')
% end


%% Third trajectory (orthongal line)

%% Helper functions


function interpolatedWaypoints = interp_waypoints(waypoints, totalPoints)
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
    interpolatedPositions = spline(normalizedCumulativeDistances, positions', tPositions)';

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
    interpolatedWaypoints = [interpolatedPositions, interpolatedQuaternions(:,[4,2,3,1])];
end

function interpolatedWaypoints = interp_waypoints_fixed_distance(waypoints, spacing)
    % Extract positions and quaternions
    positions = waypoints(:, 1:3);
    quaternions = waypoints(:, 4:7);

    % Generate a dense spline for positions
    tDense = linspace(0, 1, 10000); % Use a dense sampling to approximate the continuous path
    densePositions = spline(linspace(0, 1, size(waypoints, 1)), positions', tDense)';

    % Sample points at fixed distance apart
    [sampledPositions, sampledIndices] = samplePointsAtFixedDistance(densePositions, spacing);

    % Sample quaternions corresponding to the sampled position indices
    % Normalize quaternions before interpolation to ensure they represent valid rotations
    normQuaternions = quaternions ./ vecnorm(quaternions, 2, 2);
    denseQuaternions = interpQuaternions(normQuaternions, length(tDense), 'slerp');
    sampledQuaternions = denseQuaternions(sampledIndices, :);

    % Combine sampled positions and quaternions
    interpolatedWaypoints = [sampledPositions, sampledQuaternions];
end

function [sampledPositions, sampledIndices] = samplePointsAtFixedDistance(positions, spacing)
    sampledPositions = positions(1, :); % Start with the first point
    sampledIndices = 1; % Index of the first point
    currentDist = 0; % Initialize current distance accumulator
    for i = 2:size(positions, 1)
        currentDist = currentDist + norm(positions(i, :) - positions(i-1, :));
        if currentDist >= spacing
            sampledPositions = [sampledPositions; positions(i, :)];
            sampledIndices = [sampledIndices; i];
            currentDist = 0; % Reset distance accumulator
        end
    end
end

function interpolatedQuaternions = interpQuaternions(quaternions, numPoints, method)
    t = linspace(0, 1, size(quaternions, 1));
    tDense = linspace(0, 1, numPoints);
    interpolatedQuaternions = zeros(numPoints, 4);
    for i = 1:numPoints-1
        tFraction = (tDense(i) - t(1)) / (t(end) - t(1));
        idx = find(t <= tDense(i), 1, 'last');
        if idx < size(quaternions, 1)
            qStart = quaternions(idx, :);
            qEnd = quaternions(idx + 1, :);
            interpolatedQuaternions(i, :) = quatinterp(qStart, qEnd, tFraction, method);
        else
            interpolatedQuaternions(i, :) = quaternions(end, :);
        end
    end
    interpolatedQuaternions(end, :) = quaternions(end, [4 2 3 1]); % Ensure the last quaternion is directly assigned
end

