% Define folder names
camFolders = {'cam1', 'cam2', 'cam3'};
outputFolder = 'stitchedGIFs';  % Folder where you want to save the stitched GIFs
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);  % Create output folder if it doesn't exist
end

% Loop through each camera folder
for i = 1:length(camFolders)
    fprintf("stitching cam %d\n", i);
    camFolder = fullfile(pwd, camFolders{i});  % Full path to camera folder
    
    % Get list of all GIFs in the folder
    gifFiles = dir(fullfile(camFolder, '*.gif'));
    
    % Initialize variables for combining frames and delays
    combinedFrames = [];
    combinedDelays = [];
    colormap = [];  % Colormap for all GIFs (assumed to be the same for all GIFs)
    
    % Loop through each GIF file in the folder
    for j = 1:length(gifFiles)
        fprintf("Working on gif %s...\n", gifFiles(j).name);

        % Read the current GIF
        [frames, map, delays] = readGif(fullfile(camFolder, gifFiles(j).name));
        
        % Store the colormap from the first GIF (assuming same for all)
        if j == 1
            colormap = map;
        end

        % Ensure frames are within valid 8-bit range
        frames = uint8(255 * mat2gray(frames));  % Normalize to 0-255 range

        
        % Append the frames and delays to the combined list
        combinedFrames = cat(4, combinedFrames, frames);
        combinedDelays = [combinedDelays, delays];
    end
    
    % Create output GIF file name
    outputGif = fullfile(outputFolder, sprintf('stitched_%s.gif', camFolders{i}));
    
    % Write the combined GIF to output file
    imwrite(combinedFrames(:,:,1,1), colormap, outputGif, 'gif', 'Loopcount', inf, 'DelayTime', combinedDelays(1));
    for k = 2:size(combinedFrames, 4)
        imwrite(combinedFrames(:,:,1,k), colormap, outputGif, 'gif', 'WriteMode', 'append', 'DelayTime', combinedDelays(k));
    end
end

% Function to read GIF frames and extract delays
function [frames, colormap, delays] = readGif(filename)
    info = imfinfo(filename);  % Get GIF information
    numFrames = numel(info);   % Number of frames in the GIF

    frames = [];               % Initialize frames
    delays = zeros(1, numFrames); % Initialize delays
    colormap = [];             % Initialize colormap (first GIF frame)

    for i = 1:numFrames
        if mod(i,10) == 0
            fprintf("frame %d\n", i);
        end
        
        [frame, map] = imread(filename, 'Index', i); % Read each frame
        frames(:,:,1,i) = frame;  % Store each frame
        delays(i) = info(i).DelayTime; % Store the delay time for each frame
        if i == 1
            colormap = map;  % Set colormap from the first frame
        end
    end
end