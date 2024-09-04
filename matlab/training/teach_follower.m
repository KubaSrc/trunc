clear all; close all; clc
addpath('./util')

%% Initialize hardware

% Hardware initialization
arm = robotArm();
arm.min_motor = -250;
arm.max_motor = 150;
motor = armMotor();

% Reset arm and home
arm.reset_arm();
arm = arm.set_relative_home();

%% Import neural network

% Obtain user path
homeDir = userpath;
parts = strsplit(homeDir, '\');
homeDir = strjoin(parts(1:3), '\');

% Obtain aux-net path
pyDir = pwd;
parts = strsplit(pyDir, '\');
pyDir = strjoin(parts(1:end-2), '\');

% Set the Python interpreter if necessary
pyenv('Version', [homeDir,'/AppData/Local/anaconda3/envs/env/python.exe']);
insert(py.sys.path, int32(0), [pyDir,'/']);

% Import the Python module
auxModule = py.importlib.import_module('bot_DNN');
bot_DNN = auxModule.aux_bot_DNN(pyargs('drive_path', py.str([pyDir,'/']), ...
                                      'pos_path', py.str('matlab/training/data/2024_02_19_21_08_57/positions_norm_full.csv'), ...
                                      'train_forward', py.False, ...
                                      'train_inverse', py.False, ...
                                      'normalize', py.True));

% bot_DNN.inverse_prediction('matlab/training/inference/valve_trajectory.mat','/matlab/training/inference/DNN_valve_trajectory_inputs.mat')

%% Run main loop

follow_teacher(arm,bot_DNN)

arm.reset_arm();

%% Wrapper functions

function tool_pos = update_arm(arm)
    % Continuous processing can be placed here
    tool_pos = arm.get_pose_absolute();
end

function follow_teacher(arm,bot_DNN)
    % Create a figure for detecting key presses
    figure;

    % Set the initial 'CurrentCharacter' to a dummy value
    set(gcf, 'CurrentCharacter', '0');

    % Variable to control the loop
    finish = false;

    % Start the continuous loop
    while ~finish
       
        tool_pos = update_arm(arm);
        l_delta = bot_DNN.direct_inverse_prediction(pyargs('pos_data',py.list(tool_pos), ...
                                                    'comp',py.True, ...
                                                    'comp_path', py.str('matlab/training/calibration/cal_2024_08_05_10_39_31.mat')));
        disp(l_delta)

        arm.set_pos_delta(l_delta)

        % Check for key press
        k = get(gcf, 'CurrentCharacter');
        if k ~= '0'  % Has the character changed from the dummy character?
            set(gcf, 'CurrentCharacter', '0');  % Reset the character

            % Process the key as required
            if k == 'q'
                finish = true;  % Set finish to true to exit the loop
            end
        end

        pause(0.25);  % Short pause to reduce CPU usage and allow GUI processing
    end

    disp('You have exited the loop.');
end
