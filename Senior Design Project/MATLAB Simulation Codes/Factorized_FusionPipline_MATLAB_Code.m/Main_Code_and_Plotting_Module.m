clc;clear
% Loading the datasets
camerasynthetic= readmatrix('yolo_detections.xlsx');
radarsynthetic = readmatrix('radar_log.xlsx');
gt_data = readmatrix('camera_with_markers.xlsx');

%% Parametrs %%

Z_projection_YOLO = 0.019;

R_new = [    0.9994   -0.0043   -0.0341;
   -0.0342   -0.0068   -0.9994;
    0.0040    1.0000   -0.0069];

T_new = [-0.1509;    0.0360  ;  0.0705];

K_intrinsic = [4053.60854628705	-12.6845378485635	1979.89094280910;
                0	4046.29446186445	1577.19501797494;
                0	0	1 ];

cam_meas = camerasynthetic(:,10:11);

YOLO_W = 640;YOLO_L = 416;

pix_error = 54.316;

% Radar parametrs 
range_res = 0.293; %(m)
rvel_res = 0.31;%(m/s)
azimuth_res = pi/6; %(rad)

% Acceleration std
std_ax = sqrt(2); std_ay = sqrt(2);

% tuned parametr, 7.815 is based on the chi square distribution (alpha = 0.05 % , n =3)
G_cam = 5.991;  
G_rad = 7.815*(1+0.107); 

% Kalman Filter Initilization 
INIT_COV= 2*diag([4 4 5.65 5.65]);INIT_VELOCITY =0.6;

last_tracker_update = 0;del_threshold = 70;nStates = 4;  
Tracks(1).states_priori = [];
Tracks(1).states_posteriori = [0.883295714855194;	3.95602703094482; -0.5; -0.7];
Tracks(1).P_priori = [];
Tracks(1).P_posteriori = 2*eye(4);
Tracks(1).H = [];
Tracks(1).R = [];
Tracks(1).assigned_meas = [];
Tracks(1).status = 'active';

Tracks(1).deletion_increment =0;
Tracks(1).assignment_count = 0;

% History to save track's states every frame 
maxSteps = size(radarsynthetic,1);
maxTracks = 50;
state_history = zeros(maxSteps, maxTracks, nStates);



%% Radar data procassing 

% Determines indices corresponding to 
% the statis radar measument, to be removed

det = size(radarsynthetic,1); 
static_meas = [];
K_cam = [];

for y = 1:det
    if (radarsynthetic(y,9)==0)
        static_meas = [static_meas; y];
    end
end


% Keeps the frame count while replacing the static measurments with NaN
% such that they are no procassed 

for k = 1:length(static_meas)

    radarsynthetic(static_meas(k),5) = NaN;
    radarsynthetic(static_meas(k),6) = NaN;
    radarsynthetic(static_meas(k),8) = NaN;
end


%%%%%%%%%%%%%%%%%%%%%%% Main Structure %%%%%%%%%%%%%%%%%%%%


% The function outputs the cordinate transformed YOLO dataset and the GT
% data set 
[x_y,Rcam, gt_data] = Coordinate_Transformation_YOLO_GT(cam_meas,gt_data, Z_projection_YOLO, K_intrinsic,YOLO_W,YOLO_L, R_new, T_new, pix_error);


it = 1;

% the function process and sorts the data in terms of arrival, flagging
% measuments as either to be combined (camera and radar), or radar-only, or
% camera-only in updating the EKF states estimations 

[data_sorted] = sensor_data_process(camerasynthetic,radarsynthetic);


frame_set = unique(data_sorted(:,4));

for k = 1:length(frame_set) 

    % 4th row lists the blocks of radar and/or camera data considered every EKF predict / update iteration

    indices = find(data_sorted(:,4) == frame_set(k));
    flag = data_sorted(indices(1),5); % 3 for combine, 2 for cam, 1 for radar

    if (flag == 4) 
        % if no measurment is recieved, increment the deletion count and contnue, 
        % and the prediction and update steps are skipped no

        tracks_num = length(Tracks);
        for j = 1:tracks_num
            if (Tracks(j).deletion_increment >=del_threshold)
                Tracks(j).status = 'inactive';
            else
                Tracks(j).deletion_increment = Tracks(j).deletion_increment + 1;    
                Tracks(j).status = 'partially inactive';               
            end  
        end
        continue
    else
 
        % this function updates the measument strust with the currently
        % considered radar and/or camera measuments for iteration k, 
        % and the each measurment's noise covariance matrix for radar data

        [measurment, Rmat] = measument_struct_n_R_computing(data_sorted,indices,k, range_res, rvel_res,azimuth_res, flag);

        % the function computes delta_t, F, Q, and computes the P_priori matrix and
        % states_prriori vector 
        [last_tracker_update,Tracks, xk_] = PredictEKF(Tracks, indices,data_sorted,last_tracker_update, std_ax, std_ay);

        % calculates the jacobian of the H matrix, the
        % measuments recieved are x, y, and vr for radar, states
        % are x, y, vx, and vy, respictievly 

        [Tracks] = Hrad(Tracks, measurment, k);
        Hcam = [1 0 0 0; 0 1 0 0];

        % measument prediction step as part of the target tracking logic

        [Tracks] = measPredict(Tracks,flag, Hcam);

        % radar cost funuction 

        if (flag == 2) || (flag == 3)

            [Cost_function_rad, active_tracks, row1] = RadCostFunc(Tracks,measurment, k, G_rad, Rmat, flag);
   
        end

        % camera cost funuction

        if (flag == 1) || (flag == 3)  % camera only flag or combine flag

            [Cost_function_cam, active_tracks2, row2] = CameraCostFunc(Tracks,measurment, k, G_cam, Rcam, flag, Hcam);

        end

        % Tracks not assigned with empty ellipsoids are removed from the cost function, 
        % such that the GNN does not mistakenly assign it a measument outside its ellipsoid


        if  (flag == 1)

            [Cost_function_cam,Tracks, active_tracks2] = UnassignedTracksMangment(Cost_function_cam,Tracks, row2, active_tracks2);
            
        elseif (flag == 3)
            
            [Cost_function_cam,Tracks, active_tracks2] = UnassignedTracksMangment(Cost_function_cam,Tracks, row2, active_tracks2);
  
            [Cost_function_rad,Tracks, active_tracks] = UnassignedTracksMangment(Cost_function_rad,Tracks, row1, active_tracks);
  
        else 

            [Cost_function_rad,Tracks, active_tracks] = UnassignedTracksMangment(Cost_function_rad,Tracks, row1, active_tracks);
 
        end

   
% Hungarian Algorithm and Assignment Problem 

            if (flag == 3)

                [Tracks, matching_rad, matching_cam] = CombinedMeasToTrackAssignment(Tracks,active_tracks, measurment, k,Hcam, Rcam, Cost_function_cam, Cost_function_rad, Rmat);

            elseif (flag == 1)

                [Tracks, matching_cam] = CamMeasToTrackAssignment(Cost_function_cam,active_tracks2, Tracks,measurment, k, Rcam, Hcam);
            else

                 [Tracks, matching_rad] = RadarMeasToTrackAssignment(Cost_function_rad, Tracks,Rmat, k, measurment, active_tracks);
            end


        % intilization for unassigned measuments 

        if (flag == 3)

           [Tracks] = CameraUnassignedMeasInitialization(Tracks,measurment, matching_cam, k,INIT_VELOCITY, INIT_COV, Rcam, Hcam);

           [Tracks] = CameraUnassignedMeasInitialization(Tracks,measurment, matching_cam, k,INIT_VELOCITY, INIT_COV, Rcam, Hcam);
           
        elseif (flag == 1)

            [Tracks] = CameraUnassignedMeasInitialization(Tracks,measurment, matching_cam, k,INIT_VELOCITY, INIT_COV, Rcam, Hcam);
    
        else 
            [Tracks] = CameraUnassignedMeasInitialization(Tracks,measurment, matching_cam, k,INIT_VELOCITY, INIT_COV, Rcam, Hcam);

        end

        %% Kalman Filter Update Module 

        nTracks = length(Tracks);

        if (data_sorted(indices(1), 10) == 0)

            [Tracks,state_history] = EKFUpdate(data_sorted, Tracks,state_history, k);
        else 
            [state_history,Tracks] = OOSM_EKF_Update(Tracks, Hcam, state_history, xk_ , k);
        end
        Rmat = []; to_be_deleted=[]; 
    end
end
%% === Plotting: this module was developed using the assistance of AI ===

video_filename = 'tracker_radar_with_YOLO.mp4';
trackLineWidthBase = 1.5;
trackLineWidthMax  = 4;
fadeWindow         = 15;
fps                = 15;
min_assignments    = 30;
xyLim              = [-1 6];

%% === Load Data ===
radar_data = readmatrix('radar_log.xlsx');
frames     = unique(radar_data(:,2));
xCol = 6; yCol = 7; zCol = 8; vrCol = 9;

% Assuming gt_data is in workspace with columns:
%  col3 = timestamp, col4 = ID, col5 = x, col6 = y
% YOLO video
yoloVid = VideoReader('yolo_output.mp4');

%% === Preprocess GT (per-ID) ===
% Extract GT columns
gt_times_all = gt_data(:,3);
gt_ids_all   = gt_data(:,4);
gt_x_all     = gt_data(:,5);
gt_y_all     = gt_data(:,6);

unique_gt_ids = unique(gt_ids_all);
num_gt_ids = numel(unique_gt_ids);

% Build per-id cell arrays with ABSOLUTE timestamps (do NOT normalize)
gt_id_time = cell(num_gt_ids,1);
gt_id_x    = cell(num_gt_ids,1);
gt_id_y    = cell(num_gt_ids,1);
gt_start_times = zeros(num_gt_ids,1);
gt_end_times = zeros(num_gt_ids,1);

for i=1:num_gt_ids
    id = unique_gt_ids(i);
    mask = gt_ids_all == id;
    % sort by absolute time for safety
    [t_sorted, idxs] = sort(gt_times_all(mask));
    gt_id_time{i} = t_sorted;           % keep absolute times
    gtx = gt_x_all(mask);
    gty = gt_y_all(mask);
    gt_id_x{i} = gtx(idxs);
    gt_id_y{i} = gty(idxs);
    % record start/end in same absolute time base
    if ~isempty(t_sorted)
        gt_start_times(i) = t_sorted(1);
        gt_end_times(i)   = t_sorted(end);
    else
        gt_start_times(i) = Inf;
        gt_end_times(i)   = -Inf;
    end
end

%% === Setup figure and axes ===
fig_width = 1500; fig_height = 1000;
fig = figure('Color','w','Position',[100 100 fig_width fig_height]);
tiledlayout(2,3,'Padding','compact','TileSpacing','compact');

axFusion  = nexttile(1);
axRadarXY = nexttile(2);
axRadarRV = nexttile(3);
axYOLO    = nexttile(4);
axRMSE    = nexttile(5);
axGT      = nexttile(6);

%% === Video Writer ===
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = fps;
open(v);

% Variables from tracker logic
num_steps = size(state_history,1);
T_final   = length(Tracks);
rmse_values = nan(num_steps,1);

%% === Fix all axes initial settings ===
xlim(axFusion,xyLim); ylim(axFusion,xyLim); axis(axFusion,'equal');
xlim(axRadarXY,xyLim); ylim(axRadarXY,xyLim); axis(axRadarXY,'equal');
xlim(axRMSE,[1 num_steps]); ylim(axRMSE,[0 2]); % initial RMSE limits

%% === YOLO Frame Count & timing ===
numYoloFrames = floor(yoloVid.Duration * yoloVid.FrameRate);
yolo_time = (0:numYoloFrames-1)/yoloVid.FrameRate;

%% === Time vectors and master end time (shortest of radar, GT, YOLO) ===
radar_time = (frames - 1)/fps;  % radar timestamps in seconds

% Use max GT end time among IDs (absolute)
gt_max_time = max(gt_end_times);

% determine earliest final time among datasets (Q1 = C)
end_time = min([radar_time(end), gt_max_time, yolo_time(end)]);

% Build time_vector from radar_time but clipped to end_time (radar driving)
time_vector = radar_time(radar_time <= end_time);
if isempty(time_vector)
    error('No overlapping time region between radar, GT, and YOLO. Check timestamps.');
end
nSteps = min(num_steps, numel(time_vector));

%% === Base colors for tracks and GT IDs ===
base_colors = lines(T_final);
gt_colors = lines(max(num_gt_ids,3)); % ensure at least a few colors

%% === Main animation loop (radar-driven, stops at shortest dataset end_time) ===
for tidx = 1:nSteps
    current_time = time_vector(tidx);

    % --- Current GT positions per ID (interpolated to current_time) ---
    gt_curr_pos = nan(num_gt_ids,2); % rows: [x, y]
    for i=1:num_gt_ids
        tvec = gt_id_time{i};
        if isempty(tvec)
            continue;
        end

        % Only define a current position if the GT has started
        if current_time < gt_start_times(i)
            % not started yet -> leave NaN
            gt_curr_pos(i,:) = [NaN, NaN];
        else
            % if within ID's recorded range -> interpolate
            if current_time <= gt_end_times(i)
                gx = interp1(tvec, gt_id_x{i}, current_time, 'linear');
                gy = interp1(tvec, gt_id_y{i}, current_time, 'linear');
                gt_curr_pos(i,:) = [gx, gy];
            else
                % after ID's last time -> use last known position (keep it visible)
                gt_curr_pos(i,:) = [gt_id_x{i}(end), gt_id_y{i}(end)];
            end
        end
    end

    %% --- Fusion / Tracker subplot ---
    cla(axFusion); hold(axFusion,'on');

    rmse_sum = 0; rmse_count = 0;

    for p = 1:T_final
        if ~isfield(Tracks(p),'assignment_count') || Tracks(p).assignment_count < min_assignments
            continue;
        end

        % Extract states up to this radar-driven step
        xs = squeeze(state_history(1:tidx,p,1));
        ys = squeeze(state_history(1:tidx,p,2));

        validIdx = find(~isnan(xs) & ~isnan(ys) & ~(xs==0 & ys==0));
        if numel(validIdx) < 2, continue; end

        % --- Fading trail ---
        fade_len = min(fadeWindow, numel(validIdx));
        idx_seg  = validIdx(end-fade_len+1:end);
        fade_alpha = linspace(0.2,1,fade_len);
        base_color = base_colors(p,:);

        for k = 1:fade_len-1
            seg_color = (1-fade_alpha(k))*[0.7 0.7 0.7] + fade_alpha(k)*base_color;
            plot(axFusion, xs(idx_seg(k:k+1)), ys(idx_seg(k:k+1)), ...
                '-', 'Color', seg_color, ...
                'LineWidth', trackLineWidthBase + fade_alpha(k)*(trackLineWidthMax-trackLineWidthBase));
        end

        % Current tracker point (use most recent valid)
        cur_x = xs(idx_seg(end));
        cur_y = ys(idx_seg(end));
        scatter(axFusion, cur_x, cur_y, 60, base_color, 'filled');

        % RMSE: distance to nearest available GT current point
        if ~all(isnan(gt_curr_pos(:)))
            dists = hypot(gt_curr_pos(:,1) - cur_x, gt_curr_pos(:,2) - cur_y);
            dists(isnan(dists)) = [];  % remove NaNs
            if ~isempty(dists)
                rmse_sum = rmse_sum + dists(1)^2; % nearest GT squared distance
                rmse_count = rmse_count + 1;
            end
        end
    end

    hold(axFusion,'off');
    xlim(axFusion, xyLim); ylim(axFusion, xyLim); axis(axFusion,'equal'); grid(axFusion,'on');
    title(axFusion,sprintf('Tracker Output [Time %.2f s (step %d/%d)]', current_time, tidx, nSteps));

    %% --- Radar plots ---
    cla(axRadarXY); cla(axRadarRV);
    hold(axRadarXY,'on'); hold(axRadarRV,'on');

    pts_idx = max(1, min(tidx, numel(frames)));  % safe mapping to available frames index
    points = radar_data(radar_data(:,2)==frames(pts_idx),:);
    if ~isempty(points)
        x = points(:,xCol); y = points(:,yCol); vr = points(:,vrCol);
        rng = sqrt(x.^2 + y.^2 + points(:,zCol).^2);

        scatter(axRadarXY, x, y, 50, vr, 'filled');
        xlabel(axRadarXY,'X [m]'); ylabel(axRadarXY,'Y [m]'); grid(axRadarXY,'on');
        title(axRadarXY,sprintf('Radar Top-Down (Frame %d)',frames(pts_idx)));

        scatter(axRadarRV, rng, vr, 50, 'filled');
        xlabel(axRadarRV,'Range [m]'); ylabel(axRadarRV,'Radial Velocity [m/s]'); grid(axRadarRV,'on');
        title(axRadarRV,'Range vs Velocity');

        % keep RV axis stable
        xlim(axRadarRV, [0 max(rng)*1.1]);
    end
    hold(axRadarXY,'off'); hold(axRadarRV,'off');
    xlim(axRadarXY, xyLim); ylim(axRadarXY, xyLim);

    %% --- YOLO subplot ---
    cla(axYOLO);
    % Compute YOLO frame index safely and clamp
    yolo_idx = round(current_time * yoloVid.FrameRate) + 1;
    yolo_idx = max(1, min(yolo_idx, numYoloFrames));
    yoloFrame = read(yoloVid, yolo_idx);
    imshow(yoloFrame, 'Parent', axYOLO);
    title(axYOLO, sprintf('YOLO Detection (frame %d/%d)', yolo_idx, numYoloFrames));

    %% --- RMSE subplot ---
    if rmse_count > 0
        rmse_values(tidx) = sqrt(rmse_sum / rmse_count);
    else
        rmse_values(tidx) = NaN;
    end

    cla(axRMSE); hold(axRMSE,'on');
    plot(axRMSE, 1:tidx, rmse_values(1:tidx), 'r-', 'LineWidth',1.5);
    scatter(axRMSE, tidx, rmse_values(tidx), 40, 'r', 'filled');
    xlabel(axRMSE,'Step'); ylabel(axRMSE,'RMSE'); grid(axRMSE,'on');
    xlim(axRMSE,[1 nSteps]);
    rmse_max = max(rmse_values(1:tidx), [], 'omitnan');
    if isempty(rmse_max) || isnan(rmse_max) || rmse_max==0
        rmse_max = 1;
    end
    ylim(axRMSE, [0 rmse_max*1.1]);
    title(axRMSE,'RMSE vs Time');
    hold(axRMSE,'off');

    %% --- GT subplot (per-ID colors) ---
    cla(axGT); hold(axGT,'on');
    for i=1:num_gt_ids
        tvec = gt_id_time{i};
        if isempty(tvec)
            continue;
        end

        % Only plot history if this GT has started
        if current_time >= gt_start_times(i)
            past_mask = tvec <= current_time;
            if any(past_mask)
                xhist = gt_id_x{i}(past_mask);
                yhist = gt_id_y{i}(past_mask);
                plot(axGT, xhist, yhist, '--', 'LineWidth', 1.2, 'Color', gt_colors(i,:));
            end
        end

        % Plot current point if defined (we set NaN if not started)
        if ~isnan(gt_curr_pos(i,1))
            scatter(axGT, gt_curr_pos(i,1), gt_curr_pos(i,2), 60, gt_colors(i,:), 'filled');
            text(axGT, gt_curr_pos(i,1), gt_curr_pos(i,2), sprintf(' ID %d', unique_gt_ids(i)), 'VerticalAlignment','bottom');
        end
    end
    xlabel(axGT,'GT X [m]'); ylabel(axGT,'GT Y [m]');
    title(axGT,'Ground Truth Paths (by ID)');
    xlim(axGT, xyLim); ylim(axGT, xyLim); axis(axGT,'equal'); grid(axGT,'on');
    hold(axGT,'off');

    %% --- Render and save frame ---
    drawnow;                       % allow event processing and ensure figure updates
    frame = getframe(fig);
    writeVideo(v, frame);
end

%% --- Close video safely ---
close(v);
disp(['Video saved to ', video_filename]);

