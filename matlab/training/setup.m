clear all;
addpath('./util/NatNet_SDK_4.1/NatNetSDK/Samples/Matlab');
addpath('./util/')

home_zero = load("home_comp.mat").home_comp;
% home_zero = zeros([12,1]);

% Connect to servos and initialize the arm
if ~exist('port','var')
    port = serialport("COM4", 9600);
    channels = 0:11;
    pos = zeros(12,1);
    global position;

    initialize_servos(port, channels, 1,pos,home_zero);
    
    pause(5);
    
    stop_servos(port, channels);
end
 
%% Find position and compression value

home = [-105,-110,-90,-114,-110,-74,-116,-110,-102,0,0,0];

% Compression ratios
c_0 = 2;
c_1 = 1.5;
c_2 = 1;

% Compression scalar
c = 0;
comp = [repmat([c_0*c,c_1*c,c_2*c],1,3),[0,0,0]];
home_comp = home + comp;

for motor = 0:11
    set_servo_position_auxarm(port,motor,home_comp(motor+1),position);
end

%% Simple check for Optitrack

% Connect to motive
if ~exist('nnc','var')
    nnc = connect_to_natnet();
end

tool = nnc.getFrame().RigidBodies(1) %End Effector

%% Quick repetability test

home_comp = load("home_comp.mat").home_comp;
trial_pos = home_comp + [-170,-120,-60,0,0,0,0,0,0,0,0,0];

n_trials = 1;

% Connect to motive
if ~exist('nnc','var')
    nnc = connect_to_natnet();
end

% Store data
x_home = zeros([1,5]);
y_home = zeros([1,5]);
z_home = zeros([1,5]);

for i = 1:n_trials

    % Go to home position
    for motor = 0:11
    set_servo_position_auxarm(port,motor,home_comp(motor+1),position);
    end
    
    pause(2);

    if i == 1
        tool = nnc.getFrame().RigidBodies(1) %End Effector
        x_first = tool.x;
        y_first = tool.y;
        z_first = tool.z;
    end

    % Go to trial pos
    for motor = 0:11
    set_servo_position_auxarm(port,motor,trial_pos(motor+1),position);
    end

    pause(5);

    % % Go back to home position
    % for motor = 0:11
    % set_servo_position_auxarm(port,motor,home_comp(motor+1),position);
    % end
    
    pause(5);

    tool = nnc.getFrame().RigidBodies(1) %End Effector
    x_home(i) = tool.x;
    y_home(i) = tool.y;
    z_home(i) = tool.z;
end

d = sqrt((x_home - x_first).^2 + (y_home - y_first).^2 + (z_home - z_first).^2).*1000

%% Save the state

save('./home',"home");
save('./home_comp',"home_comp");
save('./comp_ratio',"c_0","c_1","c_2");

%% Reset motors

home_comp = load("home_comp.mat").home_comp;

% Turn off servos
initialize_servos(port, channels, 1, pos,home_comp);
pause(5)
stop_servos(port, channels);