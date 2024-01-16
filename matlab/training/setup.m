clear all;
addpath('./util');

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

% home = [-68,-75,-69,-69,-72,-71,-84,-76,-70,0,0,0];
e = -8;
r = 0;
home = [-35+e-r,-41,-42,-38.5+e+r,-38,-0,-39+e+r,-37,-37,0,0,0];

% Compression ratios
c_0 = 2;
c_1 = 1.6;
c_2 = 1;

% Compression scalar
c = 0;
comp = [repmat([c_0*c,c_1*c,c_2*c],1,3),[0,0,0]];
home_comp = home + comp;

for motor = 0:11
    set_servo_position_auxarm(port,motor,home_comp(motor+1),position);
end

%%
if ~exist('nnc','var')
    nnc = connect_to_natnet();
end

tool = nnc.getFrame().RigidBodies(1) %End Effector
x = {tool.x, tool.y, tool.z};
q = {tool.qx, tool.qy, tool.qz, tool.qw};


%% Save the state

save('./home',"home");
save('./home_comp',"home_comp");
save('./comp_ratio',"c_0","c_1","c_2");

%% Test different positions

% Parameters
de_theta = deg2rad(0);
de_phi = deg2rad(0);
dtm_theta = deg2rad(0);
dtm_phi = deg2rad(0);
dl = 0;

motor_deltas = [de_theta,de_phi,dtm_theta,dtm_phi,dl];
T = array2table(motor_deltas);
T.Properties.VariableNames(1:5) = {'de_theta','de_phi','dtm_theta','dtm_phi','dl'};
motor_pos = delta2pos(T);

for motor = 1:9
    set_servo_position_auxarm(port,motor,motor_pos(motor),position);
end

%% Reset motors

home_comp = load("home_comp.mat").home_comp;

% Turn off servos
initialize_servos(port, channels, 1, pos,home_comp);
pause(5)
stop_servos(port, channels);
