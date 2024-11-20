% Initialize parameters
weightFileName = '1000gAB';
date = '2024_11_09_16_32_42';
cam1 = fullfile(weightFileName, date, 'pictures', 'cam_1');
cam2 = fullfile(weightFileName, date, 'pictures', 'cam_2');
cam3 = fullfile(weightFileName, date, 'pictures', 'cam_3');
nFrames = 500; % Number of frames in the GIF

% Store camera folders in a cell array
camFolders = {cam1, cam2, cam3};

for i = 1:3
    fprintf("Creating GIF for Camera %d...\n",i);
    
    % Create output filname
    gif_file = sprintf('%s_cam%d_100frames.gif',weightFileName,i);
    camera = sprintf('cam%d',i);
    output_gif = fullfile('training_gifs',camera,gif_file);

    % %loop through images for cam3 250g
    % gif_file3 = sprintf('%s_cam%d_100frames.gif',weightFileName,3);
    % camera3 = sprintf('cam%d',3);
    % output_gif3 = fullfile('training_gifs',camera3,gif_file3);
    % for y = 1:100
    %         imageName = sprintf('trajectory%d_pose_%d__cam%d.jpg', 1, y,3);
    %         img = imread(fullfile(camFolders{3},imageName));
    % 
    %         % Example resizing the image to 50% of the original size
    %         resize_factor = 0.5;  % Change this to scale as needed
    %         img_resized = imresize(img, resize_factor);
    % 
    %         % Convert to indexed image with 256 colors (GIF format requirement)
    %         [imind, cm] = rgb2ind(img_resized, 256);  
    % 
    %         % Write the image to the GIF file
    %         if y == 1
    %             % For the first frame, create the GIF file
    %             imwrite(imind, cm, output_gif3, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
    %         else
    %             % Append subsequent images to the GIF file
    %             imwrite(imind, cm, output_gif3, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
    %         end
    % end

    for x = 1:1
        for y = 1:100
            imageName = sprintf('trajectory%d_pose_%d.jpg', x, y);
            img = imread(fullfile(camFolders{i},imageName));
            
            % Example resizing the image to 50% of the original size
            resize_factor = 0.5;  % Change this to scale as needed
            img_resized = imresize(img, resize_factor);

            % Convert to indexed image with 256 colors (GIF format requirement)
            [imind, cm] = rgb2ind(img_resized, 256);  

            % Write the image to the GIF file
            if x == 1 && y == 1
                % For the first frame, create the GIF file
                imwrite(imind, cm, output_gif, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
            else
                % Append subsequent images to the GIF file
                imwrite(imind, cm, output_gif, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
            end
        end
    end
end