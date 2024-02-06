clear all; close all; clc
addpath('../training/util')

% Test variables
pause_length = 0.5;
pulse_length = 0;
noise_samples = 15;
trajectory = 'circle';

%% Initial setup

% Save path
currentDateTime = datetime('now');
dirName = datestr(currentDateTime, '_yyyy_mm_dd_HH_MM_SS');
save_path = ['./experiments/',trajectory, dirName];
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
l_delta = load('./circle/trajectory.mat').output;
comp = load('../training/state/comp.mat').comp;


%% Loop and collect data

% Initialize output
output = cell(num_points*repeat + 1, 9);
output(1,:) = {'date and time','p',...
    'x_end_avg','y_end_avg','z_end_avg','qx_end_avg','qy_end_avg','qz_end_avg','qw_end_avg'};

fprintf('Starting test\n');

% Run test loop
for r = 1:repeat
    fprintf('Repetition %d/%d\n', r, repeat);
   
    % Determine the order of points
    if random_order
        % Visit points in a random order
        pointOrder = randperm(num_points);
    else
        % Visit points in a fixed order
        pointOrder = 1:num_points;
    end

    arm.reset_arm();

    for p_idx = 1:num_points
        
        p = pointOrder(p_idx); % Get the actual point index

        fprintf('Point: %d/%d Point id: %d \n', p_idx, num_points, p);
        fprintf('========================\n');
        
        % Set arm to new pose
        arm.set_pos(comp+l_delta(p,:))
        
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
        output((r-1)*num_points+p+1,1:4) = {datetime,r,p,p_idx};
        output((r-1)*num_points+p+1,5:11) = num2cell(mean(S,1));

        % Take a photo
        img = getsnapshot(cam);
        % Save the image to disk.
        filename = sprintf('/pose_%d_repetition_%d.jpg', p, r);
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