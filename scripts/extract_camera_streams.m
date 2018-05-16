%% doc
% This script extracts all images and disparity maps from the given bag
% file.
% The disparity map is post processed to a clean disparity map (we get rid
% of debug information).
% Frame parameters are stored in a csv file with collumns 
% "focallength, baseline, ccx, ccy"
% Optionaly, one can set "extract_point_clouds" to true. This will
% create a point cloud for each stream and merge them into one large
% point cloud. 
% Each step is written to disk according to the given path "output_path".

close all;
clear;

%% read config.m script
% If you don't have one, copy config_example.m, rename it to config.m
% and set the paths accordingly
config;

tic;

cam_color{1} = 'red';
cam_color{2} = 'blue';
cam_color{3} = 'green';
cam_color{4} = 'yellow';
cam_color{5} = 'black';
cam_color{6} = 'cyan';
cam_color{7} = 'magenta';
cam_color{8} = 'red';

%% start script

%read calibration file
camchain = YAML.read(camchain_path);

%calculate minor rotation correction (R_0-R_3) based on calibration params
R = undistrect(camchain);

T_cn_cnm{1} = eye(4);
T_cn_cnm{2} = camchain.cam1.T_cn_cnm1;
T_cn_cnm{3} = camchain.cam2.T_cn_cnm1;
T_cn_cnm{4} = camchain.cam3.T_cn_cnm1;
T_cn_cnm{5} = camchain.cam4.T_cn_cnm1;
T_cn_cnm{6} = camchain.cam5.T_cn_cnm1;
T_cn_cnm{7} = camchain.cam6.T_cn_cnm1;
T_cn_cnm{8} = camchain.cam7.T_cn_cnm1;

% read rosbag
fprintf('Reading bag...\n');
bag = rosbag(rosbag_path);

% create ouput path if it doesn't exist
if 0 == exist(output_path, 'dir')
    mkdir(output_path);
end

% write file containing parameters R and T_cn_cnm
% Note: reshapes linearizes columns ([1 2; 3 4] -> [1 3 2 4])
out_R = [output_path '/params_R.csv'];
Rout = zeros(streams, 16);
for i = 1:streams
    Rout(i, :) = reshape(R{i}, 1, 16);
end
csvwrite(out_R, Rout);

out_camchain = [output_path '/params_camchain.csv'];
ccOut = zeros(2*streams, 16);
for i = 1:(2*streams)
    ccOut(i,:) = reshape(T_cn_cnm{i}, 1, 16);
end
csvwrite(out_camchain, ccOut);

% extract camera image messages
img_streams(1) = select(bag, 'Topic', '/uvc_baseboard0/cam_0/image_rect');
img_streams(2) = select(bag, 'Topic', '/uvc_baseboard0/cam_0/image_depth');
img_streams(3) = select(bag, 'Topic', '/uvc_baseboard0/cam_2/image_rect');
img_streams(4) = select(bag, 'Topic', '/uvc_baseboard0/cam_2/image_depth');
img_streams(5) = select(bag, 'Topic', '/uvc_baseboard1/cam_0/image_rect');
img_streams(6) = select(bag, 'Topic', '/uvc_baseboard1/cam_0/image_depth');
img_streams(7) = select(bag, 'Topic', '/uvc_baseboard1/cam_2/image_rect');
img_streams(8) = select(bag, 'Topic', '/uvc_baseboard1/cam_2/image_depth');

%extract camera info messages
cams{1} = select(bag, 'Topic', '/uvc_baseboard0/cam_0/camera_info');
cams{2} = select(bag, 'Topic', '/uvc_baseboard0/cam_2/camera_info');
cams{3} = select(bag, 'Topic', '/uvc_baseboard1/cam_0/camera_info');
cams{4} = select(bag, 'Topic', '/uvc_baseboard1/cam_2/camera_info');



% find common image range
lowestIndex = min([img_streams.NumMessages]);
highestIndex = max([img_streams.NumMessages]);

% hope the first image exists to preallocate our array
msg = readMessages(img_streams(1), 1);
pic1 = readImage(msg{1});
[h, w] = size(pic1);

endIndex = min(lowestIndex, startFrame + maxProcessFrames-1);

fprintf('start: %d, last: %d\n', startFrame, endIndex);

for image_index = startFrame:endIndex
    % preallocate
    out_pics = cell(streams, 1);
    out_depths = cell(streams, 1);
    out_depthsCleaned = cell(streams, 1);
    out_ptCloud = cell(streams, 1);
    pics = cell(streams, 1);
    depths = cell(streams, 1);
    depthsCleaned = cell(streams, 1);
    focallengths = zeros(streams, 1);
    baselines = zeros(streams, 1);
    ccx = zeros(streams, 1);
    ccy = zeros(streams, 1);
    % initialize point clouds
    xyzPoints = cell(streams, 1);

    progress = ['Extracting: ' int2str(image_index) '/' int2str(lowestIndex) ];
    fprintf(progress);
    
    % create output file names
    f = int2str(image_index);
    for i = 1:streams
        s = int2str(i);
        out_pics{i} = [output_path '/pic' '_s_' s '_f_' f '.png' ];
        out_depths{i} = [output_path '/disparity' '_s_' s '_f_' f '.png' ];
        out_depthsCleaned{i} = [output_path '/disparity_normalized' '_s_' s '_f_' f '.png' ];
        out_ptCloud{i} = [output_path '/point_cloud' '_s_' s '_f_' f '.ply' ];
    end
    out_frame_params = [output_path '/params_f_' f '.csv'];
    out_ptCloudTotal = [output_path '/point_cloud_total_f_' f '.ply'];

    % some logic to skip frames we've already processed
    skip = exist(out_frame_params, 'file') && (exist(out_ptCloudTotal, 'file') || ~extract_point_clouds);
    for i = 1:streams
        if  ~exist(out_pics{i}, 'file') ...
            || (export_raw_disparity && ~exist(out_depths{i}, 'file')) ...
            || ~exist(out_depthsCleaned{i}, 'file') ...
            || (~exist(out_ptCloud{i}, 'file') && extract_point_clouds)
            skip = false;
            break;
        end
    end

    % perform skip of frame, or not.
    if skip && use_cache
        fprintf(repmat('\b', 1, numel(progress)));
        continue;
    end
    

    % core processing: extract images, disparity maps, parameters
    for i = 1:streams
        if img_streams(2*i-1).NumMessages < image_index
            % missing data, skip
            continue;
        end
        %get camera instrinsics and baseline values from camera info messages
        camMsg = readMessages(cams{i},image_index);
        focallengths(i) = camMsg{1}.K(1);
        baselines(i) = -T_cn_cnm{2*i}(1,4);
        ccx(i) = camMsg{1}.K(3);
        ccy(i) = camMsg{1}.K(6);
    
        % read intensity image
        msg = readMessages(img_streams(2*i-1), image_index);
        pics{i} = readImage(msg{1});
        % read disparity map
        msgDepth = readMessages(img_streams(2*i), image_index);
        depths{i} = readImage(msgDepth{1});
        % process disparity map to get a cleaned version
        % crop last 3 bits of 8 bit disparity value (contains debug info)
        disparityMap = bitand(depths{i},248);%248 = 11111000bin
        %adjust for extended disparity range with offset of 32 pixels and every 2nd
        %sample only
        disparityMap = single((uint8(disparityMap))/8.0);
        disparityMap = disparityMap*2+32;
        depthsCleaned{i} = disparityMap;
    end

    % write first batch of data
    for i = 1:streams
        if img_streams(2*i-1).NumMessages < image_index
            % missing data, skip
            continue;
        end
        imwrite(pics{i}, out_pics{i});
        if export_raw_disparity
            imwrite(depths{i}, out_depths{i});
        end
        cleaned = uint8(depthsCleaned{i});
        imwrite(cleaned, out_depthsCleaned{i});
    end
    % write file containing parameters focallengths, baselines, ccx, ccy
    M = [focallengths baselines ccx ccy];
    csvwrite(out_frame_params, M);
    
    % also calculate each point cloud per disparity map
    % also calculate total point cloud
    if extract_point_clouds
        %avoid division 0, crop last 4 disparity values
        mindisp = 1*2+32;
        %set constant to replace division 0
        scaleddisp = 2*2+32*10000;

        %compensate for x/y offset of disparity map
        xshift = 22;
        yshift = -8;
        %crop image boarder regions
        frame = 80;
        yRange = (1+frame):1:(h-frame);
        xRange = (1+frame):1:(w-frame);
        for i = 1:streams
            if img_streams(2*i-1).NumMessages < image_index
                % missing data, skip
                continue;
            end
            xyzPoints{i} = single(zeros(h,w,3));
            T = eye(4);
            for iter = 2:i
                T = T_cn_cnm{2*iter-1}*T_cn_cnm{2*iter-2}*T;
            end
            % convert each pixel to a point in the point cloud (without
            % border)
            for y=yRange
                for x=xRange
                    if(depthsCleaned{i}(y,x,1) < mindisp)
                        xyzPoints{i}(y,x,3) = focallengths(i)*baselines(i)/scaleddisp;
                    else
                        xyzPoints{i}(y,x,3) = focallengths(i)*baselines(i)/(depthsCleaned{i}(y,x,1));
                    end
                    xyzPoints{i}(y,x,1) = (x+xshift-ccx(i))/focallengths(i)*xyzPoints{i}(y,x,3);
                    xyzPoints{i}(y,x,2) = (y+yshift-ccy(i))/focallengths(i)*xyzPoints{i}(y,x,3);

                    %rotate/translate 3D points to cam rect reference frame
                    temp = R{i}*T\[xyzPoints{i}(y,x,1),xyzPoints{i}(y,x,2),xyzPoints{i}(y,x,3),1 ]';
                    xyzPoints{i}(y,x,1) = temp(1);
                    xyzPoints{i}(y,x,2) = temp(2);
                    xyzPoints{i}(y,x,3) = temp(3);
                end
            end
        end
        % concatenate point clouds to get one large cloud
        xyzPointsTotal = [];
        for i = 1:streams
            if img_streams(2*i-1).NumMessages < image_index
                % missing data, skip
                continue;
            end
            xyzPointsTotal = [xyzPointsTotal, xyzPoints{i}];
        end

        % write point clouds
        for i = 1:streams
            if img_streams(2*i-1).NumMessages < image_index
                % missing data, skip
                continue;
            end
            ptCloud = pointCloud(xyzPoints{i});
            pcwrite(ptCloud, out_ptCloud{i}, 'PLYFormat', 'binary');
        end

        ptCloud = pointCloud(xyzPointsTotal);
        pcwrite(ptCloud, out_ptCloudTotal, 'PLYFormat', 'binary');
    end
    
    % show some progress by displaying the processed frame
    if display
        figure(1);
        %show figure with extracted rect and depth images
        for i = 1:streams
            subplot(2,4,i)
            imshow(pics{i});
            subplot(2,4,i+4)
            imshow(depths{i});
        end
        if extract_point_clouds
            % show point cloud (if we calculated it)
            figure(2);
            pcshow(xyzPoints{1});
            hold on;
            for i = 2:streams
                pcshow(xyzPoints{i});
            end

            T = cell(2*streams, 1);
            acc = T_cn_cnm{1};
            T{1} = acc;
            for i = 2:(2*streams)
                acc = T_cn_cnm{i} * acc;
                T{i} = inv(acc);
            end

            %plot camera positions in 3d view
            size_cam = 0.012;
            for i = 1:(2*streams)
                plotCamera('Location',T{i}([1 2 3],4),'Orientation',T{i}([1 2 3],[1 2 3])','Opacity',0,'Size',size_cam,'Color',cam_color{i});
            end

            hold off;
        end
        % give the gui a chance to redraw the figure
        pause(0.1);
    end
    % reset our progress bar in the console
    fprintf(repmat('\b', 1, numel(progress)));
end

fprintf('\nFinished.\n');

toc;
