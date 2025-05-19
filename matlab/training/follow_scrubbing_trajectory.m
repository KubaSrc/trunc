clear all; close all; clc
addpath('./util')

% Test variables

pause_length = 0.45; 
pulse_length = 0;
noise_samples = 15;
model_type = 'DNN';
trajectory = 'circle';
record = false;
demo = false;
active_pause = false;
active_motor = false;

surface = 'toilet';
inputs_path = ['./inference/scrubbing/',surface,'_inputs.mat']; % Motor positions from model
pause_path = ['./inference/scrubbing/',surface,'_pause.mat']; % Motor positions from model
motor_path = ['./inference/scrubbing/',surface,'_motor.mat']; % Motor positions from model

% Loading trajectory info for approach
l_delta = load(inputs_path).output;
pause_mat = load(pause_path).pause_mat;
motor_mat = load(motor_path).motor_mat;
num_points =size(l_delta,1);
comp = load('./state/comp.mat').comp;


%% Initial setup

% Hardware initialization
arm = robotArm();
arm.min_motor = -250;
arm.max_motor = 150;
motor = armMotor();

%% Loop and collect data


fprintf('Starting test\n');

motor_on = false;

% Loop to initial position
for p = 1:num_points

    % Check for hover
    if pause_mat(p) > 1
        pause(pause_length*pause_mat(p));
        fprintf('Long pause %0.1f\n',pause_length*pause_mat(p))
    else
        pause(pause_length);
    end

    arm.set_pos_delta(l_delta(p,:))

    % Pause based on commands
    if pause_mat(p) == 1
        input("Press enter to continue");
    end

    if motor_mat(p) && ~motor_on
        motor.toggle();
        motor_on = true;
        disp("Turning motor on");
    elseif ~motor_mat(p) && motor_on
        motor.toggle();
        motor_on = false;
        disp("Turning motor off");
    end

end

motor.toggle();