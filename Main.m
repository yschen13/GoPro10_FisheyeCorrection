% This is a MATLAB script to detect the camera model of GoPro10 and remove distortions from videos

cd('~/Documents/CV/')

%{
calibration_file = 'GX010057.MP4';
v = VideoReader(calibration_file);
frame = read(v,3300);
[imagePoints,boardSize] = detectCheckerboardPoints(frame,'HighDistortion',true);
f = figure;
% I = insertMarker(I, imagePoints(:,:,i), 'o', 'Color', 'red', 'Size', 10);
imshow(frame);
saveas(f,['Checkerboard.png'])
imds = imageDatastore(fullfile(toolboxdir('vision'),'visiondata','calibration','gopro'));
[imagePoints,boardSize,imagesUsed] = detectCheckerboardPoints(imds.Files(1:4),'HighDistortion',true);

[imagePoints,boardSize] = detectCheckerboardPoints('GOPR0059.JPG','HighDistortion',true);
%}


% generate calibration pictures from the video
califile = 'GX010121.MP4';
v = VideoReader(califile);
frame_idx = 1:100:1101;
for i = 1:length(frame_idx)
	frame1 = read(v,frame_idx(i));
	imwrite(frame1,['tag36_lowres/' num2str(i) '.png'])
end


% Calibration using AprilTagCorner
tagArrangement = [5,8];
tagFamily = 'tag36h11';


% Create an imageDatastore object to store the captured images.
% imdsCalib = imageDatastore("aprilTagCalibImages/");
imdsCalib = imageDatastore('tag36_lowres/');

% Detect the calibration pattern from the images.
[imagePoints, boardSize] = helperDetectAprilTagCorners(imdsCalib, tagArrangement, tagFamily);

idx = 1;
I = readimage(imdsCalib, idx);
[tagIds, tagLocs] = readAprilTag(I, tagFamily);


markerRadius = 8;
numCorners = size(tagLocs,1);
for i = 1:size(tagLocs,3)
	markerPosition = [tagLocs(:,:,i),repmat(markerRadius,numCorners,1)];
	I = insertShape(I,"FilledCircle",markerPosition,"Color","red","Opacity",1);
end

f = figure;
imshow(I)
saveas(f,'test.png')


tagSize = 20; % mm
worldPoints = generateCheckerboardPoints(boardSize, tagSize);

% Determine the size of the images.
I = readimage(imdsCalib, 1);
imageSize = [size(I,1), size(I,2)];

% Estimate the camera parameters.
params = estimateFisheyeParameters(imagePoints, worldPoints, imageSize);

% Display the reprojection errors.
f = figure;
showReprojectionErrors(params)
saveas(f,'ProjectionError.png')


% Read a calibration image.
idx = 1;
I = readimage(imdsCalib, idx);

% Insert markers for the detected and reprojected points.
I = insertMarker(I, imagePoints(:,:,idx), 'o', 'Color', 'g', 'Size', 10);
I = insertMarker(I, params.ReprojectedPoints(:,:,idx), 'x', 'Color', 'r', 'Size', 10);

% Display the image.
f = figure;
imshow(I)
saveas(f,'test2.png')

% Display the extrinsics.
f = figure;
showExtrinsics(params)
saveas(f,'Extrinsics.png')

idx = 2;
I = readimage(imdsCalib,idx);
J2 = undistortFisheyeImage(I,params.Intrinsics);
f = figure;
imshow(J2)
title('Output View with low Scale Factor')
saveas(f,'undistortFisheyeImage.png')





% Read in actual movie
file = 'GX010056_p1.mov';
v = VideoReader(file);
frame = read(v,100);
J2 = undistortFisheyeImage(frame, params.Intrinsics,'OutputView','same');
imwrite(J2,'undistort.png')

f = figure;
imshowpair(frame,J2,'montage');
saveas(f,['Compare.png'])


% undistort the entire video
new_v = VideoWriter('GX010056_p1_undistort.avi');
new_v.FrameRate = v.FrameRate;
open(new_v)
for i = 1:v.NumFrames
	frame = read(v,i);
	J = undistortFisheyeImage(frame, params.Intrinsics,'OutputView','valid');
	writeVideo(new_v,J)
	if mod(i,100) == 0
		i
	end
end
close(new_v)











%{
%% Generate the calibration pattern

downloadURL  = 'https://github.com/AprilRobotics/apriltag-imgs/archive/master.zip';
dataFolder   = fullfile(tempdir, 'apriltag-imgs', filesep); 
options      = weboptions('Timeout', Inf);
zipFileName  = [dataFolder, 'apriltag-imgs-master.zip'];
folderExists = exist(dataFolder, 'dir');

% Create a folder in a temporary directory to save the downloaded file.
if ~folderExists  
    mkdir(dataFolder); 
    disp('Downloading apriltag-imgs-master.zip (60.1 MB)...') 
    websave(zipFileName, downloadURL, options); 
    
    % Extract contents of the downloaded file.
    disp('Extracting apriltag-imgs-master.zip...') 
    unzip(zipFileName, dataFolder); 
end


% Set the properties of the calibration pattern.
tagArrangement = [5,8];
tagFamily = 'tag36h11';

% Generate the calibration pattern using AprilTags.
tagImageFolder = [dataFolder 'apriltag-imgs-master/' tagFamily];
imdsTags = imageDatastore(tagImageFolder);
calibPattern = helperGenerateAprilTagPattern(imdsTags, tagArrangement, tagFamily);

%} 