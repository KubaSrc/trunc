clear all; close all; clc
addpath('./util')

%% Initial setup

% Save path
currentDateTime = datetime('now');
dirName = datestr(currentDateTime, '_yyyy_mm_dd_HH_MM_SS');
save_path = ['./record/path', dirName];

mkdir(save_path);

% Hardware initialization
arm = robotArm();
motor = armMotor();

% Constants
noise_samples = 15;

%% Loop and collect data

% Initialize output
output = cell2table([{""},num2cell(zeros(1,9))],'VariableNames',{'date and time','p',...
    'x_end_avg','y_end_avg','z_end_avg','qx_end_avg','qy_end_avg','qz_end_avg','qw_end_avg'});

fprintf('Collecting\n');
   
arm.reset_arm();

collect = true;
p = 0;

disp('Press "Enter" to keep running and -1 to exit.')

while collect

    s = input(['Point: ', num2str(p),'\n']);

    % Check exit condition
    if s == -1
        break
    end

    % Increment point count
    p = p+1;

    % Sample to reduce noise
    S = zeros(noise_samples,7);
    
    for i = 1:noise_samples
        tool = arm.get_pose();
        S(i,:) = [tool.x, tool.y, tool.z, tool.qx, tool.qy, tool.qz, tool.qw];
        pause(1/60);
    end
    
    % Record actuation duraction
    if isnumeric(s) && isscalar(s) && s > 0
        output.actuation(p) = s;
    end

    % Write to output
    output(p,1:2) = {datetime,p};
    output(p,3:9) = num2cell(mean(S,1));
end


% Clean up.
arm.reset_arm();

% Save output file

writetable(output,[save_path,'/positions.csv'])
copyfile('./state', save_path);