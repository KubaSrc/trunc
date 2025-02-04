clear all; close all; clc
addpath('./util')

% Test variables
pause_length = 3;
pulse_length = 0;
noise_samples = 15;
movment_time = 2;
traj_start = 1;
anti_slack = true;
slack_spacing = 2;
randomized = true;

%% Initial setup

weight = '-50g_random/';

% Save path
currentDateTime = datetime('now');
dirName = datestr(currentDateTime, 'yyyy_mm_dd_HH_MM_SS');
save_path = ['./data/',weight, dirName];
photo_path = [save_path,'/pictures'];
mkdir(save_path);
mkdir(photo_path);
mkdir([photo_path,'/cam_1']);
mkdir([photo_path,'/cam_2']);
mkdir([photo_path,'/cam_3']);

% Save relevant info files
copyfile('./state', save_path);
copyfile('./trajectory', save_path);

% Camera setup
cam_1 = videoinput('winvideo', 1);
set(cam_1, 'FramesPerTrigger', Inf);
set(cam_1, 'ReturnedColorspace', 'rgb')
cam_1.FrameGrabInterval = 1;  % Grab one frame every second

cam_2 = videoinput('winvideo', 2);
set(cam_2, 'FramesPerTrigger', Inf);
set(cam_2, 'ReturnedColorspace', 'rgb')
cam_2.FrameGrabInterval = 1;  % Grab one frame every second

cam_3 = videoinput('winvideo', 3);
set(cam_3, 'FramesPerTrigger', Inf);
set(cam_3, 'ReturnedColorspace', 'rgb')
cam_3.FrameGrabInterval = 1;  % Grab one frame every second

% Hardware
arm = robotArm(1);
% motor = armMotor();
comp = load("./state/comp.mat").comp;

%% Loop and collect data

training_waypoints = load('./trajectory/training_waypoints.mat').points;

num_points = size(training_waypoints,1);
% num_trajectories = size(training_waypoints,3);
num_trajectories = 20;

% Initialize output
output = cell(num_points*num_trajectories + 1, 19);
output(1,:) = {'date and time','Trajectory','Waypoint',...
    'x_end_avg','y_end_avg','z_end_avg','qx_end_avg','qy_end_avg','qz_end_avg','qw_end_avg',...
    'l0','l1','l2','l3','l4','l5','l6','l7','l8'};

fprintf('Starting test\n');
fprintf('Estimated test duration: %0.3f hours\n',  num_points * (pause_length + movment_time) * num_trajectories/ 3600);

% Save header
writecell(output(1,:),[save_path,'/positions.csv'])

% Run test loop
for r = traj_start:num_trajectories
    fprintf('Trajectory %d/%d\n', r, num_trajectories);

    arm.reset_arm();

    slack = zeros([1,9]);

    if randomized
        r = randi(size(training_waypoints,3));
    end

    fprintf('Random Trajectory Index: %d\n', r);

    for p = 1:num_points

        fprintf('Point: %d/%d\n', p,num_points);
        fprintf('========================\n');
        delta_pos = training_waypoints(p,:,r); 
        pos = comp+training_waypoints(p,:,r);

        % Anti-slack compensation
        slack_idx = [3,2,1,6,5,4,9,8,7];
        p_t = 2;
        r_t = 0.75;


        % Set arm to new position
        disp("Moving arm.")
        slack = 0.5.*slack; 
        arm.set_pos(pos+slack)

        % Wait for it to settle
        settled = false;
        max_settle_time = 15; % Maximum time to wait for settling (in seconds)
        threshold = 0.75; % Threshold for movement (in mm)
        rolling_window_time = 1; % Time window to calculate movement (in seconds)
        sample_rate = 0.1; % Sampling interval (in seconds)
        max_samples = rolling_window_time / sample_rate; % Maximum samples in the rolling window
        
        poses = []; % Initialize array to store rolling window of positions
        start_time = tic; % Start timer
        

        while ~settled
            % Get the current pose of the arm
            pose_i = arm.get_pose();
            
            % Extract the x, y, z position
            position_i = 1000.*[pose_i.x, pose_i.y, pose_i.z];
            
            % Append the current position to the rolling window
            poses = [poses; position_i];
            
            % Ensure rolling window size does not exceed max_samples
            if size(poses, 1) > max_samples
                poses(1, :) = []; % Remove oldest position
            end
            
            % Calculate movement if we have enough samples
            if size(poses, 1) == max_samples
                % Compute the maximum movement within the rolling window
                max_movement = max(vecnorm(diff(poses, 1, 1), 2, 2));
                
                % Check if the movement is below the threshold
                if max_movement < threshold
                    settled = true;
                    disp('Arm has settled.');
                end
            end
            
            % Check for timeout
            if toc(start_time) > max_settle_time
                disp('Timeout reached without settling.');
                break;
            end
            
            % Pause briefly to control the sample rate
            pause(sample_rate);
        end

        % if anti_slack && mod(p-1,slack_spacing) == 0
        if anti_slack
            % Loop through each cable
            disp('Removing slack')
            for i = 1:length(slack_idx)
                slack_removed = false;
                pause(0.25)
                tool_0 = arm.get_pose();
                while ~slack_removed
                    idx = slack_idx(i);
                    slack(idx) = slack(idx)-.075;
                    arm.set_pos(pos+slack)
                    tool_i = arm.get_pose();
                    slack_removed = check_movment(tool_0,tool_i,p_t,r_t);
                end
            end
        end

        % Set arm to new pose
        arm.set_pos(pos+slack)
        
        % Pause for equillibirum
        pause(pause_length)


        % Sample to reduce noise
        S = zeros(noise_samples,7);
        
        for i = 1:noise_samples
            tool = arm.get_pose();
            S(i,:) = [tool.x, tool.y, tool.z, tool.qx, tool.qy, tool.qz, tool.qw];
            pause(1/60);
        end
        
        % Write to output
        output((r-1)*num_points+p+1,1:3) = {datetime,r,p};
        output((r-1)*num_points+p+1,4:10) = num2cell(mean(S,1));
        output((r-1)*num_points+p+1,11:19) = num2cell(delta_pos+slack);
        writecell(output((r-1)*num_points+p+1,:),[save_path,'/positions.csv'],'WriteMode','append')

        % Take a photo
        img_1 = getsnapshot(cam_1);
        % Save the image to disk.
        filename = sprintf('/cam_1/trajectory%d_pose_%d.jpg', r, p);
        im_path = [photo_path,filename];
        imwrite(img_1, im_path);

        img_2 = getsnapshot(cam_2);
        % Save the image to disk.
        filename = sprintf('/cam_2/trajectory%d_pose_%d.jpg', r, p);
        im_path = [photo_path,filename];
        imwrite(img_2, im_path);

        img_3 = getsnapshot(cam_3);
        % Save the image to disk.
        filename = sprintf('/cam_3/trajectory%d_pose_%d.jpg', r, p);
        im_path = [photo_path,filename];
        imwrite(img_3, im_path);

    end
end

% Reset arm and camera
arm.reset_arm();

delete(cam_1)
clear cam_1

delete(cam_2)
clear cam_2

delete(cam_3)
clear cam_3

%% Helper function

function d = quaternionDistance(q1, q2)
    % Ensure the quaternions are normalized
    q1 = q1 / norm(q1);
    q2 = q2 / norm(q2);

    % Calculate the dot product (cosine of the angle)
    dotProd = abs(dot(q1, q2));

    % Calculate the distance
    d = 1 - dotProd;
end


function slack_removed = check_movment(t_0,t_i,p_t,r_t)
slack_removed = false;

p_0 = 1000.*[t_0.x,t_0.y,t_0.z];
p_i = 1000.*[t_i.x,t_i.y,t_i.z];

r_0 = [t_0.qw,t_0.qx,t_0.qy,t_0.qz];
r_i = [t_i.qw,t_i.qx,t_i.qy,t_i.qz];

d = sqrt(sum((p_0-p_i).^2));
d_r = rad2deg(quaternionDistance(r_0,r_i));

if d >= p_t || d_r >= r_t
    slack_removed = true;
end

end