close all; clc; clear all;
addpath('./util')

save_output = false;

%%

arm_model = trunc_model();

arm_model.update_config([60,0,...
                         60,0,...
                         60,0],0);

[~,delta_segments] = arm_model.find_segments()
delta_segments = arm_model.find_adjusted_segments()
% arm_model.find_lengths()
% % arm_model.draw_arm()