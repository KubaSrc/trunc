clear all; close all; clc
addpath('../training/util')

% Test variables
pause_length = 0.5;
pulse_length = 0;
noise_samples = 15;
trajectory = 'circle';
trajectory_name = ['./',trajectory,'_trajectory.mat'];
inputs_name = ['./',trajectory,'_trajectory_inputs.mat'];

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
l_delta = load(inputs_name).output;
num_points=size(l_delta,1);
comp = load('../training/state/comp.mat').comp;

copyfile(trajectory_name, save_path);
copyfile(inputs_name, save_path);

%% Loop and collect data

% Initialize output
output = cell(num_points*repeat + 1, 9);
output(1,:) = {'date and time','p',...
    'x_end_avg','y_end_avg','z_end_avg','qx_end_avg','qy_end_avg','qz_end_avg','qw_end_avg'};

fprintf('Starting test\n');


   
arm.reset_arm();

for p = 1:num_points
   

    fprintf('Waypoint: %d/%d \n', p, num_points);
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
    output((r-1)*num_points+p+1,1:2) = {datetime,p};
    output((r-1)*num_points+p+1,3:9) = num2cell(mean(S,1));

    % Take a photo
    img = getsnapshot(cam);
    % Save the image to disk.
    filename = sprintf('/pose_%d_repetition_%d.jpg', p);
    im_path = [photo_path,filename];
    imwrite(img, im_path);

end


% Reset
arm.reset_arm();

% Clean up.
delete(cam)
clear cam

% Save output file
writecell(output,[save_path,'/positions.csv'])
copyfile('../training/state', save_path);
copyfile('../training/trajectory', save_path);