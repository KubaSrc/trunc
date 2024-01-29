clear all; close all; clc
addpath('./util')

% Test variables
pause_length = 1.75;
random_order = true;
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

% Camera setup
cam = videoinput('winvideo', 1);
set(cam, 'FramesPerTrigger', Inf);
set(cam, 'ReturnedColorspace', 'rgb')
cam.FrameGrabInterval = 1;  % Grab one frame every second

% Hardware
arm = robotArm();
motor = armMotor();
l_delta = load("./trajectory/delta_fast_repeat.mat").delta_fast;
comp = load("./state/comp.mat").comp;

%% Loop and collect data

training_waypoints = load('./trajectory/training_waypoints.mat').training_waypoints;

num_points = size(training_waypoints,1);
num_trajectories = size(training_waypoints,3);

% Initialize output
output = cell(num_points*repeat + 1, 10);
output(1,:) = {'date and time','Trajectory','Waypoint',...
    'x_end_avg','y_end_avg','z_end_avg','qx_end_avg','qy_end_avg','qz_end_avg','qw_end_avg'};

fprintf('Starting test\n');
fprintf('Total %d samples\n', repeat);
fprintf('Estimated test duration: %0.3f hours\n',  num_points * (pause_length + movment_time) * num_trajectories/ 3600);

% Run test loop
for r = 1:num_trajectories
    fprintf('Trajectory %d/%d\n', r, num_trajectories);

    arm.reset_arm();

    for p = 1:num_points

        fprintf('Point: %d/%d', p);
        fprintf('========================\n');
        
        % Set arm to new pose
        arm.set_pos(comp+training_waypoints(p,:,r))
        
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

        % Take a photo
        img = getsnapshot(cam);
        % Save the image to disk.
        filename = sprintf('/pose_%d_trajectory_%d.jpg', p, r);
        im_path = [photo_path,filename];
        imwrite(img, im_path);

    end
end

% Reset
arm.reset_arm();

% Clean up.
delete(cam)
clear cam

% Save output file
writecell(output,[save_path,'/positions.csv'])
copyfile('./state', save_path);
copyfile('./trajectory', save_path);