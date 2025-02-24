close all; clc; clear all;

home_triad_pos = load('./state/home_triad_measured.mat').pos;
addpath('./util/NatNet_SDK_4.1/NatNetSDK/Samples/Matlab');
addpath('./util/')

home_triad_pos = load('./state/home_triad_measured.mat').pos;

export_traj = true;

% DEFINE F_arm and F_tool
F_arm = (583./1000).*9.81;
F_tool = ((234+580)./1000).*9.81; % Counter rotating brush + Mass A

% Find position of plate
nnc = connect_to_natnet();
bodies = nnc.getFrame().RigidBodies;

y_plate = 0.2361*1000;
y_calibrate = 0;
y_offset = 1000.*(0.1241-y_calibrate); % m
delta_x = -25;

% CHECK THESE INDICIES WHEN DEFINING NEW RIGID BODIES
arm = bodies(2);
plate = bodies(1);

%% Define scrubbing position 
X = [home_triad_pos(1)+delta_x,y_plate+y_offset,home_triad_pos(3)];
Q = [0,0,0,1];
XQ = [X,Q];


Fn = 0:0.25:10;
Fg = -F_arm - F_tool;
Ft = -Fg - Fn;
% Ft = [15.3428,12.84129,9.60399,7.15149,6.19011];


%% Trajectory to move brush to position

wp = [home_triad_pos;XQ];
wp =  interp_waypoints(wp,100,"linear");
wp(:,4:end) = repmat([0,0,0,1],[size(wp,1),1]);

Ft_start = repmat(Ft(1),[size(wp,1),1]);

wp = [wp,Ft_start];
wp_approach = wp;

% Apply compensations
if export_traj
    save_path = sprintf('./inference/instron_approach_dx_%d.mat',delta_x);
    save(save_path,'wp')
end

%% This is the data collection trajectory

XQ_full = [repmat(XQ,size(Ft.')),Ft.'];
wp = XQ_full;
wp_collect = wp;

% Apply compensations
if export_traj
    save_path = sprintf('./inference/instron_collect_dx_%d.mat',delta_x);
    save(save_path,'wp')
end

%% Helpfer functions

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





