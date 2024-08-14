% Auto-generated by cameraCalibrator app on 02-Aug-2024
%-------------------------------------------------------


% Define images to process
imageFileNames = {'C:\Users\hajjafar.k\aux-net\matlab\training\camCal3v2\CAM3\WIN_20240802_13_50_52_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal3v2\CAM3\WIN_20240802_13_50_55_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal3v2\CAM3\WIN_20240802_13_50_57_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal3v2\CAM3\WIN_20240802_13_51_01_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal3v2\CAM3\WIN_20240802_13_51_05_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal3v2\CAM3\WIN_20240802_13_51_15_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal3v2\CAM3\WIN_20240802_13_51_18_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal3v2\CAM3\WIN_20240802_13_51_29_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal3v2\CAM3\WIN_20240802_13_51_37_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal3v2\CAM3\WIN_20240802_13_51_40_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal3v2\CAM3\WIN_20240802_13_51_42_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal3v2\CAM3\WIN_20240802_13_51_52_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal3v2\CAM3\WIN_20240802_13_52_49_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal3v2\CAM3\WIN_20240802_13_52_59_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal3v2\CAM3\WIN_20240802_13_53_07_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal3v2\CAM3\WIN_20240802_13_53_13_Pro.jpg',...
    'C:\Users\hajjafar.k\aux-net\matlab\training\camCal3v2\CAM3\WIN_20240802_13_53_18_Pro.jpg',...
    };
% Detect calibration pattern in images
detector = vision.calibration.monocular.CheckerboardDetector();
[imagePoints, imagesUsed] = detectPatternPoints(detector, imageFileNames, 'HighDistortion', true);
imageFileNames = imageFileNames(imagesUsed);

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);

% Generate world coordinates for the planar pattern keypoints
squareSize = 23;  % in units of 'millimeters'
worldPoints = generateWorldPoints(detector, 'SquareSize', squareSize);

% Calibrate the camera
[cameraParams3, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

%Save the cameraParams into a separate file:
save('params.mat', "cameraParams3", "-append")

% % View reprojection errors
% h1=figure; showReprojectionErrors(cameraParams);
% 
% % Visualize pattern locations
% h2=figure; showExtrinsics(cameraParams, 'CameraCentric');
% 
% % Display parameter estimation errors
% displayErrors(estimationErrors, cameraParams);
% 
% % For example, you can use the calibration data to remove effects of lens distortion.
% undistortedImage = undistortImage(originalImage, cameraParams);
% 
% % See additional examples of how to use the calibration data.  At the prompt type:
% % showdemo('MeasuringPlanarObjectsExample')
% % showdemo('StructureFromMotionExample')
