%This script is to help sort the images that are not in separate folders
%per image.

% Define the source folder and destination folders
source_folder = fullfile('250g', '2024_11_06_17_21_07', 'pictures');
destination_folder_cam1 = fullfile('250g', '2024_11_06_17_21_07', 'pictures', 'cam_1');
destination_folder_cam2 = fullfile('250g', '2024_11_06_17_21_07', 'pictures', 'cam_2');
destination_folder_cam3 = fullfile('250g', '2024_11_06_17_21_07', 'pictures', 'cam_3');

% Make sure destination folders exist
if ~exist(destination_folder_cam1, 'dir')
    mkdir(destination_folder_cam1);
end
if ~exist(destination_folder_cam2, 'dir')
    mkdir(destination_folder_cam2);
end
if ~exist(destination_folder_cam3, 'dir')
    mkdir(destination_folder_cam3);
end

% Get list of all files in the source folder
file_list = dir(fullfile(source_folder, '*.jpg'));  % Adjust for file type if necessary (e.g., '*.png')

% Loop through each file and sort into the appropriate folder
for i = 1:length(file_list)
    filename = file_list(i).name;
    
    % Split the filename based on underscores
    parts = split(filename, '_');
    
    % Extract the camera name (z is the last part of the filename)
    camera_name = parts{end};
    
    % Determine the destination folder based on the camera name
    if strcmp(camera_name, 'cam1.jpg')
        movefile(fullfile(source_folder, filename), fullfile(destination_folder_cam1, filename));
    elseif strcmp(camera_name, 'cam2.jpg')
        movefile(fullfile(source_folder, filename), fullfile(destination_folder_cam2, filename));
    elseif strcmp(camera_name, 'cam3.jpg')
        movefile(fullfile(source_folder, filename), fullfile(destination_folder_cam3, filename));
    end
end