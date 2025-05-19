clear all; close all; clc;
addpath('./util/')

% Create an instance of the arm and motor
motor = armMotor();

arm = robotArm(3);
    
% arm.reset_arm()


%% Record home position

pos = [1000*0.004233654	1000*0.3243583	1000*0.04298552	-0.014648786	-0.036390118	0.007323412	0.999203471];

save('./state/home_triad_measured.mat',"pos");
 
%% Find home position

set_home = false;

home = [80,94,91,...
        55,92,81,...
        63,83,83];
arm.set_pos(home)

if set_home
    save('./state/home',"home");
end

%% Verify compression

set_comp = false;

home = load('./state/home').home;
delta_l = -30;
comp_delta = repmat(delta_l.*[1,5/7,3/7],[1,3]);
comp = home + comp_delta;

arm.set_pos(comp);

if set_comp
    save('./state/comp',"comp");
end


%% Set max compression

set_comp = false;

home = load('./state/home').home;
delta_l = -70;
comp_delta = repmat(delta_l.*[1,5/7,3/7],[1,3]);
comp_max = home + comp_delta;


arm.set_pos(comp_max);

if set_comp
    save('./state/comp_max',"comp_max");
end

%% Comp loop


home = [80,94,88,...
        60,92,81,...
        63,83,75];

delta_l = -100;
comp_delta = repmat(delta_l.*[1,5/7,3/7],[1,3]);
comp = home + comp_delta;


for i=1:10
    arm.set_pos(comp);
    arm.set_pos(home)
end


%%  Motor

motor.pulse(5)

%% Set pictures pose 1

end_pose = [
    1393.00;
    1417.50;
    1588.75;
    1290.25;
    1584.00;
    1657.25;
    1686.75;
    1701.50;
    1681.75];

arm.set_pos_target(end_pose)


%% Set pictures pose 2

end_pose = [
    1500.75;
    1642.75;
    1716.00;
    1373.50;
    1466.50;
    1569.25;
    1721.00;
    1740.50;
    1706.25];

arm.set_pos_target(end_pose)



%% Reset arm

arm.reset_arm()

%% Optitrack

tool = arm.get_pose();
disp(tool)
pos = [tool.x*1000,tool.y*1000,tool.z*1000,tool.qw,tool.qx,tool.qy,tool.qz];

%% Turn off motors

arm.stop_motors()