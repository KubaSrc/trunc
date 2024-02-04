clear all; close all; clc
addpath('./util')

% Test variables
pause_length = .5;
pulse_length = 0;
noise_samples = 15;
movment_time = 2;

%% Initial setup

% Save path
currentDateTime = datetime('now');
dirName = datestr(currentDateTime, 'yyyy_mm_dd_HH_MM_SS');
save_path = ['./data/', dirName];
photo_path = [save_path,'/pictures'];
mkdir(save_path);
mkdir(photo_path);

% Save relevant info files
copyfile('./state', save_path);
copyfile('./trajectory', save_path);

% Camera setup
cam = videoinput('winvideo', 1);
set(cam, 'FramesPerTrigger', Inf);
set(cam, 'ReturnedColorspace', 'rgb')
cam.FrameGrabInterval = 1;  % Grab one frame every second

% Hardware
arm = robotArm();
motor = armMotor();
comp = load("./state/comp.mat").comp;

%% Loop and collect data

training_waypoints = load('./trajectory/training_waypoints.mat').points;

num_points = size(training_waypoints,1);
num_trajectories = size(training_waypoints,3);

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
for r = 1:num_trajectories
    fprintf('Trajectory %d/%d\n', r, num_trajectories);

    arm.reset_arm();

    for p = 1:num_points

        fprintf('Point: %d/%d\n', p,num_points);
        fprintf('========================\n');
        
        % Set arm to new pose
        delta_pos = training_waypoints(p,:,r); 
        pos = comp+training_waypoints(p,:,r);
        arm.set_pos(pos)
        
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
        output((r-1)*num_points+p+1,11:19) = num2cell(delta_pos);
        writecell(output((r-1)*num_points+p+1,:),[save_path,'/positions.csv'],'WriteMode','append')

        % Take a photo
        img = getsnapshot(cam);
        % Save the image to disk.
        filename = sprintf('/trajectory%d_pose_%d.jpg', r, p);
        im_path = [photo_path,filename];
        imwrite(img, im_path);

    end
end

% Reset arm and camera
arm.reset_arm();

delete(cam)
clear cam