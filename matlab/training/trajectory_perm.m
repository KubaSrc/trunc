close all; clc; clear all;

rng(7);
dr_max = 50; % limit for rotation
dl_max = 60; % limit for extension
n = 20000;

num_trajectories = 200;
num_waypoints = 50;
max_trajectory_length = 200;
point_per_distance = 1/10;

%% Define sweep over the configuration space

dl = -dl_max.*rand(n,1);

% Wrist
dl_wrist = -dr_max.*rand(n, 3);
for i = 1:n
    zeroIndex = randi(3);
    dl_wrist(i, zeroIndex) = 0; % Randomly zero out to prevent a length change
end

% Elbow
dl_elbow = -dr_max.*rand(n, 3);
for i = 1:n
    zeroIndex = randi(3);
    dl_elbow(i, zeroIndex) = 0; % Randomly zero out to prevent a length change
end

% Shoulder
dl_shoulder = -dr_max.*rand(n, 3);
for i = 1:n
    zeroIndex = randi(3);
    dl_shoulder(i, zeroIndex) = 0; % Randomly zero out to prevent a length change
end


% Anti-slackening compensation
dl_elbow = dl_elbow + dl_shoulder;
dl_wrist = dl_wrist + dl_elbow;

% Add in compression values
dl_wrist = dl_wrist + dl;
dl_elbow = dl_elbow + (5/7).*dl;
dl_shoulder = dl_shoulder + (3/7).*dl;

lengths_sweep = [dl_wrist(:,1),dl_elbow(:,1),dl_shoulder(:,1),...
                dl_wrist(:,2),dl_elbow(:,2),dl_shoulder(:,2),...
                dl_wrist(:,3),dl_elbow(:,3),dl_shoulder(:,3)];