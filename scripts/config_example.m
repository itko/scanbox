% this is an example for a valid configuration file
% adjust paths according to your needs

% set these paths according to your bag file
rosbag_path = '2018-01-23-10-24-43.bag';
camchain_path = 'camchain-2018-01-22-18-51-48.yaml';

% generated files will be written here
output_path = '../data/2018-01-23-10-24-43';

% do you want to see a live preview? 
% (a window pops up with the processed images and disparity maps)
display = false;

% how many camera streams are there? (number of disparity maps)
streams = 4;

% calculating point clouds takes a moment
% you probably don't need this
extract_point_clouds = false;

% which frames should be processed? indexing starts at 1
startFrame = 1;
% if maxProcessFrames + startFrame > totalFrames, no crash will occur
% if maxProcessFrames = 0: only startFrame will be processed
maxProcessFrames = 1000000000; 

% Export raw disparity maps that include debug values and aren't scaled?
export_raw_disparity = false;

% does not overwrite complete frames on disk (but overwrites all files, if
% some are missing)
use_cache = true;
