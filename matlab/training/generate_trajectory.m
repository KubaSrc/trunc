close all; clc; clear all;

home_pos = load('./state/home_measured.mat').pos;
point_density = load('./state/point_density.mat').dist_per_point;

%% First trajectory (circle)

export_traj = false;

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
wp = interp_waypoints(wp,100,"cubic");

figure(1); clf; hold on; grid on

plot3(wp(:,1), wp(:,2), wp(:,3), 'x-', 'LineWidth', 1.5); % 'k:' makes the line black and dotted, 'LineWidth' sets the thickness

xlabel('X-axis'); % Label for the x-axis
ylabel('Y-axis'); % Label for the y-axis
zlabel('Z-axis'); % Label for the z-axis

if export_traj
    save('./inference/circle_trajectory.mat','wp')
end

%% Second trajectory (steps)

tool_rot = eye(3);
tool_quat = rotm2quat(tool_rot);

l = 80;
d = 20;
z_plane = 60;

wp_xyz = [0,0,0; % Origin
          0,l,z_plane; % #1 (start)
          0,l,z_plane-d;
          0,l,z_plane; % #1 (end)
          -l*sin(pi/6),-l*cos(pi/6),z_plane; % #2 (start)
          -l*sin(pi/6),-l*cos(pi/6),z_plane-d;
          -l*sin(pi/6),-l*cos(pi/6),z_plane; % #2 (start)
          l*sin(pi/6),-l*cos(pi/6),z_plane; % #3 (start)
          l*sin(pi/6),-l*cos(pi/6),z_plane-d;
          l*sin(pi/6),-l*cos(pi/6),z_plane; % #3 (end)
          0,l,z_plane; % #1 (start)
          ];

n = length(wp_xyz);

wp = [wp_xyz,repmat(tool_quat,[n,1])];
wp(:,1:3) = wp(:,1:3) + home_pos(1:3);
wp = interp_waypoints(wp,100,"linear");

figure(2); clf; hold on; grid on

plot3(wp(:,1), wp(:,2), wp(:,3), 'x-', 'LineWidth', 1.5); % 'k:' makes the line black and dotted, 'LineWidth' sets the thickness

xlabel('X-axis'); % Label for the x-axis
ylabel('Y-axis'); % Label for the y-axis
zlabel('Z-axis'); % Label for the z-axis

title('3D Line Plot with Grid'); % Title for the plot

if export_traj
    save('./inference/triangle_trajectory.mat','wp')
end

view([-15,30])

%% Third trajectory (orthongal line)

tool_0_rot = eye(3);
tool_0_quat = rotm2quat(tool_0_rot);

tool_1_rot = [cosd(90)  0  sind(90);
                0         1  0;
             -sind(90)  0  cosd(90)];
tool_1_quat = rotm2quat(tool_1_rot);

dx = -20;
dz = 20;
z0 = 30;
x0 = -60;

wp_xyz = [0,0,0; % Origin
          x0,0,z0; % #1 (start)
          x0+dx,0,z0;
          x0,0,z0; % #1 (start)
          x0,0,z0+dz; % #2 (start)
          x0+dx,0,z0+dz;
          x0,0,z0+dz; % #2 (end)
          x0,0,z0+2*dz; % #3 (start)
          x0+dx,0,z0+2*dz;
          x0,0,z0+2*dz; % #3 (end)
          ];

n = length(wp_xyz);

wp = [wp_xyz,repmat(tool_quat,[n,1])];
wp(:,1:3) = wp(:,1:3) + home_pos(1:3);
wp = interp_waypoints(wp,100,"linear");

figure(3); clf; hold on; grid on

plot3(wp(:,1), wp(:,2), wp(:,3), 'x-', 'LineWidth', 1.5); % 'k:' makes the line black and dotted, 'LineWidth' sets the thickness

xlabel('X-axis'); % Label for the x-axis
ylabel('Y-axis'); % Label for the y-axis
zlabel('Z-axis'); % Label for the z-axis

title('3D Line Plot with Grid'); % Title for the plot

if export_traj
    save('./inference/triangle_trajectory.mat','wp')
end

view([0,0])

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