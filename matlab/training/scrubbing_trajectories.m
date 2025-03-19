close all; clc; clear all;

home_triad_pos = load('./state/home_triad_measured.mat').pos;
addpath('./util/NatNet_SDK_4.1/NatNetSDK/Samples/Matlab');
addpath('./util/')

home_triad_pos = load('./state/home_triad_measured.mat').pos;

export_traj = true;

% DEFINE F_arm and F_tool
F_arm = (583./1000).*9.81;
F_tool = ((234+580+250)./1000).*9.81; % Counter rotating brush + Mass A
    
% Find position of plate
nnc = connect_to_natnet();
bodies = nnc.getFrame().RigidBodies;

y_plate = 0.2361*1000;
y_calibrate = -1./1000;
y_offset = 1000.*(0.1241-y_calibrate); % m
delta_x = 25;

% CHECK THESE INDICIES WHEN DEFINING NEW RIGID BODIES
arm = bodies(2);
plate = bodies(1);

%% CONSTANTS

% DEFINE F_arm and F_tool
F_arm = (583./1000).*9.81;
F_tool = ((234+580+250)./1000).*9.81; % Counter rotating brush + Mass A
Fg = -F_arm - F_tool;

%% Spiral for grill

X = [home_triad_pos(1),y_plate+y_offset,home_triad_pos(3)+delta_x];
Q = [0,0,0,1];
XQ = [X,Q];

Fn = 2;
Ft_move = -Fg;
Ft_scrub = -Fg - Fn;

% Parameters
max_diameter = 150-30;   % Maximum diameter of the spiral
num_turns = 3;       % Number of complete turns

% Derived quantities
max_radius = max_diameter / 2;         % Maximum radius from the center
theta_max = 2 * pi * num_turns;          % Maximum angle in radians

% Create a vector of theta values
theta = linspace(0, theta_max, 1000);    % 1000 points for smoothness

% Calculate the coefficient 'b' so that r(theta_max) equals max_radius
b = max_radius / theta_max;

% Compute the radius for each theta
r = b * theta;

% Convert polar coordinates (r, theta) to Cartesian coordinates (x, y)
x = r .* cos(theta);
y = r .* sin(theta);

% Plot the spiral
figure;
plot(x, y, 'LineWidth', 2);
axis equal;          % Ensure the x and y axes are scaled equally
xlabel('X');
ylabel('Y');
title('Archimedean Spiral');
grid on;


%% Cresent for toilet seat

%% Back and forth for plate

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
