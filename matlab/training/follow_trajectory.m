clear all; close all; clc

motor_testing = 1; %run motors; 0 to test code and 1
save_path = append("./demo_E1_",datestr(datetime('now'),'mm_dd_yyyy'),".csv");
pause_length = 1.5; 

home_position = load("home_position.mat").home_position;  
compress = load("compress.mat").compress; 

% Define home position with compression values
m_cs = 1.5; e_cs = 1.75; % Compression scaler
t_c = -24; m_c = m_cs*t_c; e_c = e_cs*t_c;
t_home = [-8,-34,-42,-42];
m_home = [-32,-40,-31,-41];
e_home = [-34,-40,-30,-33];
t_compressed = t_home + t_c; 
m_compressed = m_home +  m_c; 
e_compressed = e_home + e_c;

% Calculate positions from motor deltas
md = readtable('motor_trajectory_E1.csv');
traject = readtable('trajectory_E1.csv');
num_points = size(md,1);

% Convert deltas in motor positions
motor_pos_top = [-(md.dtm_13>0).*(md.dtm_13)+md.dl-abs(0.5*md.dtm_24),-(md.dtm_24>0).*(md.dtm_24)+md.dl-abs(0.5*md.dtm_13),(md.dtm_13<0).*(md.dtm_13)+md.dl-abs(0.5*md.dtm_24),(md.dtm_24<0).*(md.dtm_24)+md.dl-abs(0.5*md.dtm_13)];
motor_pos_mid = [-1.5.*(md.dtm_13>0).*(md.dtm_13)+m_cs.*md.dl-abs(0.75*md.dtm_24),-1.5.*(md.dtm_24>0).*(md.dtm_24)+m_cs.*md.dl-abs(0.75*md.dtm_13),1.5.*(md.dtm_13<0).*(md.dtm_13)+m_cs.*md.dl-abs(0.75*md.dtm_24),1.5.*(md.dtm_24<0).*(md.dtm_24)+m_cs.*md.dl-abs(0.75*md.dtm_13)];
motor_pos_end = [-(md.de_13>0).*(md.de_13)+e_cs.*md.dl-abs(0.5*md.de_24),-(md.de_24>0).*(md.de_24)+e_cs.*md.dl-abs(0.5*md.de_13),(md.de_13<0).*(md.de_13)+e_cs.*md.dl-abs(0.5*md.de_24),(md.de_24<0).*(md.de_24)+e_cs.*md.dl-abs(0.5*md.de_13)];

motor_pos_top = motor_pos_top + t_compressed;
motor_pos_mid = motor_pos_mid + m_compressed;
motor_pos_end = motor_pos_end + e_compressed;

motor_pos = [motor_pos_top(:,1),motor_pos_mid(:,1),motor_pos_end(:,1),...
            motor_pos_top(:,2),motor_pos_mid(:,2),motor_pos_end(:,2),...
            motor_pos_top(:,3),motor_pos_mid(:,3),motor_pos_end(:,3),...
            motor_pos_top(:,4),motor_pos_mid(:,4),motor_pos_end(:,4)];

% Connect to servos and initialize the arm
if ~exist('port','var')
    port = serialport("COM6", 9600);
    channels = 0:11;
    pos = zeros(12,1);
    global position;
%  
end

% Connect to NatNet
pause(1);
if ~exist('nnc','var')
    nnc = connect_to_natnet();
end

% Compress
% if motor_testing
%     start_pos = home_position;
%     for scale = linspace(0.1,1,10)
%         for motor = 0:11
%             set_servo_position_auxarm(port,motor,start_pos(motor+1)+scale*(compress(motor+1)-start_pos(motor+1)),position);
%         end
%         pause(0.15)
%     end
% end

pause(8); 

ee = nnc.getFrame().RigidBody(1); %End Effector

xe = {ee.x, ee.y, ee.z} 
qe = {ee.qx, ee.qy, ee.qz, ee.qw};

%% Loop and collect data along trajectory

% Initialize output
init_frame = nnc.getFrame();
output = cell(num_points + 1, 14); % + 3 + 4 + marker_count * 3);
output(1,:) = {'date and time','sample','dtm_13','dtm_24','de_13','de_24','dl',...
    'x_end_avg','y_end_avg','z_end_avg','qx_end_avg','qy_end_avg','qz_end_avg','qw_end_avg'};

fprintf('Starting Experiment\n');

ee = nnc.getFrame().RigidBody(1); %End Effector
point_index = traject.orig_index(1);
wp = 0;

% Increase servo speed for testing
channels = 0:11;

for channel = channels 
    set_servo_speed(port, channel, 1);
end


for p = 1:num_points
    % Verify that Optitrack is connected
    if (~nnc.IsConnected)
        error('Not connected :(');
        break;
    end

    fprintf('Point %d/%d\n', p, num_points);
    fprintf('========================\n');

    % Command each servo based on motor position for that iteration
    if motor_testing
        for motor = 0:11
            set_servo_position_auxarm(port,motor,motor_pos( p,motor+1),position);
        end
    end
      
    if traject.orig_index(p) > point_index 
        fprintf('\n###################################################\n')
        fprintf('Waypoint {%.d} reached.  Press any key to continue.',wp)
        fprintf('\n###################################################\n\n')
        pause
        wp = wp+1;
    end

    point_index = traject.orig_index(p);

    % Sample 15 times to reduce noise
    S = zeros(15,7);
    
    for i = 1:15
        % frame = nnc.getFrame();
        ee = nnc.getFrame().RigidBody(1); %End Effector
        S(i,:) = [ee.x, ee.y, ee.z, ee.qx, ee.qy, ee.qz, ee.qw];
        pause(1/30);
    end
    
    % Write to output
    output(p+1,1:2) = {datetime,p};
    output(p+1,3:7) = table2cell(md(p,:));
    output(p+1,8:14) = num2cell(mean(S,1));

    % Save every 1000 points
    if mod(p-1,1000) == 0
        writecell(output,save_path);
    end

end

% Decrease at the end 
channels = 0:11;

for channel = channels 
    set_servo_speed(port, channel, 1);
end

% initialize_servos(port, channels, pause_length,pos,home_position);
% pause(3)
% stop_servos(port, channels);

% Save output file
writecell(output,save_path);