close all; clc; clear all;

home_triad_pos = load('./state/home_triad_measured.mat').pos;
addpath('./util/NatNet_SDK_4.1/NatNetSDK/Samples/Matlab');
addpath('./util/')


    
% Find position of plate
nnc = connect_to_natnet();
bodies = nnc.getFrame().RigidBodies;

% CHECK THESE INDICIES WHEN DEFINING NEW RIGID BODIES
arm = bodies(2);
plate = bodies(1);

disp(arm.y);