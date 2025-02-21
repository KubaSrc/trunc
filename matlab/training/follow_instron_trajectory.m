clear all; close all; clc
addpath('./util')

% Test variables
pause_length = 0.5; 
pulse_length = 0;
noise_samples = 15;
model_type = 'DNN';
trajectory = 'circle';
record = false;
demo = false;
active_pause = false;
active_motor = false;

approach_inputs_path = "././inference/instron_approach_0_inputs.mat"; % Motor positions from model
collect_inputs_path = "././inference/instron_collect_0_inputs.mat"; % Motor positions from model
combined_inputs_path = "././inference/instron_combined_0_inputs.mat"; % Motor positions from model

% Loading trajectory info for approach
l_delta_approach = load(approach_inputs_path).output;
num_points_approach=size(l_delta_approach,1);
comp = load('./state/comp.mat').comp;

% Loading trajectory info for approach
l_delta_collect = load(collect_inputs_path).output;
num_points_collect=size(l_delta_collect,1);

% Loading trajectory info for approach
l_delta_combined = load(combined_inputs_path).output;
num_points_combined=size(l_delta_combined,1);



figure(1); hold on
plot(1:size(l_delta_collect,1),l_delta_collect(:,1))
plot(1:size(l_delta_approach,1),l_delta_approach(:,1))

%% Initial setup

% Hardware initialization
arm = robotArm();
arm.min_motor = -250;
arm.max_motor = 150;
motor = armMotor();

%% Loop and collect data


fprintf('Starting test\n');

% Loop to initial position
for p = 1:num_points_approach
    
    arm.set_pos_delta(l_delta_approach(p,:))

end

% Wait for plate to be installed

X = input("Press enter to continue");

% % Loop to initial position
% for p = 1:num_points_collect
% 
%     arm.set_pos_delta(l_delta_collect(p,:))
%     pause(0.25)
% 
% end
% 
% X = input("Press enter to finish test");
% 
% arm.reset_arm();

