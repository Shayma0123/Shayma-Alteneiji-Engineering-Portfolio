clc;clear
camerasynthetic= readmatrix('yolo_detections.xlsx');
radarsynthetic = readmatrix('radar_log.xlsx');
% Determines indices corresponding to
% the statis radar measument
det = size(radarsynthetic,1);
static_meas = [];
K_cam = [];
for y = 1:det
    if (radarsynthetic(y,9)==0)
        static_meas = [static_meas; y];
    end
end
R_new = [ 0.9994 -0.0043 -0.0341;
         -0.0342 -0.0068 -0.9994;
          0.0040 1.0000 -0.0069];

T_new = [-0.1509; 0.0360 ; 0.0705];

Ext = [R_new T_new];

K_intrinsic = [4053.60854628705 -12.6845378485635 1979.89094280910;
                0 4046.29446186445 1577.19501797494;
                0 0 1];
cam_meas = camerasynthetic(:,10:11);
YOLO_W = 640;YOLO_L = 416;
Orig_W = 4056;Orig_L = 3040;
cam_meas(:,1) = cam_meas(:,1)*(Orig_W/YOLO_W);
cam_meas(:,2) = cam_meas(:,2)*(Orig_L/YOLO_L);
P_projective = K_intrinsic*Ext;

Z = 0.019;

H = [P_projective(:,1) P_projective(:,2) P_projective(:,3)*Z+P_projective(:,4) ];
pixels = [cam_meas(:,1)';
          cam_meas(:,2)';
          ones(1, size(cam_meas,1))];
x_y_w = H\pixels;
x_y = (x_y_w(1:2,:) ./ x_y_w(3,:))';
x_y(:, 2) = abs(x_y(:, 2));
d_uv = [54.417; 54.417; 1];
dx_y_1 = inv(H)*d_uv;
d_x_y = dx_y_1(1:2,:)/dx_y_1(3,:);
Rcam = [d_x_y(1)^2 0 ; 0 d_x_y(2)^2];

gt_data = readmatrix('camera_with_markers.xlsx');
u_v_gt = gt_data(:, 5:7)';
Trc = -R_new*T_new ;

Rrc = R_new';
x_y_gt_nor = (Rrc*u_v_gt + Trc)';
gt_data(:, 5:7) = x_y_gt_nor;

% Keeps the frame count while replacing the static measurments with NaN
% such that they are no procassed
%radarsynthetic_new = radarsynthetic;
for k = 1:length(static_meas)
    radarsynthetic(static_meas(k),5) = NaN;
    radarsynthetic(static_meas(k),6) = NaN;
    radarsynthetic(static_meas(k),8) = NaN;
end
% Radar parametrs
range_res = 0.293; %(m)
rvel_res = 0.31;%(m/s)
azimuth_res = pi/6; %(rad)
std_ax = sqrt(2); std_ay = sqrt(2);
% Kalman Filter Initilization
G_rad = 7.815*(1+0.107);
G_rad = (1)*G_rad;
last_tracker_update = 0;del_threshold = 70;
INIT_COV= 2*diag([4 4 5.65 5.65]);INIT_VELOCITY =0.6;
nStates = 4; % number of states
Tracks(1).states_priori = [];
Tracks(1).states_posteriori = [0.883295714855194; 3.95602703094482; -0.5; -0.7];
Tracks(1).P_priori = [];
Tracks(1).P_posteriori = 2*eye(4);
Tracks(1).H = [];
Tracks(1).R = [];
Tracks(1).assigned_meas = [];
Tracks(1).status = 'active';
Tracks(1).deletion_increment =0;
Tracks(1).assignment_count = 0;
%%%%%%%%%%%%%%%%%%%%%%% Main Structure %%%%%%%%%%%%%%%%%%%%
maxSteps = size(radarsynthetic,1);
maxTracks = 50;
state_history = zeros(maxSteps, maxTracks, nStates);
it = 1;
% the function process and sorts the data in terms of arrival, flagging
% measuments as either to be combined (camera and radar), or updating EKF
% using radar only or camera only
t_cam_meas_frame = camerasynthetic(:,2);
cam_pi_clock = camerasynthetic(:,3);
cam_meas = camerasynthetic(:,10:11);

t_rad_meas_frame = radarsynthetic(:,3);
rad_meas = radarsynthetic(:,[6:7, 9]);
rad_pi_clock = radarsynthetic(:,4);
rad_meas(:,1:2) = rad_meas(:,1:2);
FPS_cam = 12;
FP_rad = 10;
%FPS =lcm(FPS_cam, FP_rad);
FPS = 1000;
radar_dt = 1/FPS;
camera_dt = 1/FPS;
t_rad_meas_frame = round(t_rad_meas_frame/radar_dt)*radar_dt;
t_cam_meas_frame = round(t_cam_meas_frame/camera_dt)*camera_dt;
data = [ t_rad_meas_frame rad_pi_clock ones(length(t_rad_meas_frame),1) zeros(length(t_rad_meas_frame),1) zeros(length(t_rad_meas_frame),1) rad_meas];
 % t_cam_meas_frame cam_pi_clock zeros(length(t_cam_meas_frame),1) zeros(length(t_cam_meas_frame),1) zeros(length(t_cam_meas_frame),1) x_y zeros(length(t_cam_meas_frame), 1)];
data_sorted = sortrows(data, 2);
%data_sorted = sortrows(data, 1);
block_num = 1;
%frame_groups = radarsynthetic(:,1);
frame_groups = findgroups(data_sorted(:,1));
% frame group column is 3
data_sorted(:,9) = frame_groups;
set = unique(frame_groups);
static_target_indices =[];
for w = 1:length(set)
    indeces_orig = find(data_sorted(:, 9) ==set(w));
    check_static_meas = isnan(data_sorted(indeces_orig, 6));
    num_recived_meas_per_fram = length(check_static_meas);
    intermediate = sum(check_static_meas(:) == 1);

    if ~(intermediate == num_recived_meas_per_fram)
        indeces = indeces_orig(~check_static_meas); % moving_target_indicces
        static_target_indices = indeces_orig(check_static_meas);

        max_pi_timestamp = max(data_sorted(indeces,2));
        min_pi_timestamp = min(data_sorted(indeces,2));

        if (max_pi_timestamp- min_pi_timestamp <0.05) && (length(unique(data_sorted(indeces,3)))>1)

            data_sorted(indeces,4) = block_num*ones(length(indeces),1);
            block_num = block_num+1;
            data_sorted(indeces,5) = 3*ones(length(indeces),1); % combine flag
        elseif (max_pi_timestamp- min_pi_timestamp <0.05) && (length(unique(data_sorted(indeces,3)))<2)
            data_sorted(indeces,4) = block_num*ones(length(indeces),1);
            block_num = block_num+1;
            if data_sorted(indeces(1),3) == 0
                data_sorted(indeces,5) = 1*ones(length(indeces),1); % camera flag
            else
                data_sorted(indeces,5) = 2*ones(length(indeces),1); % radar flag
            end
        elseif ~(max_pi_timestamp- min_pi_timestamp <0.05) && (length(unique(data_sorted(indeces,3)))>1)

            indeces0 = data_sorted(indeces, 3) == data_sorted(indeces(1), 3);
            sensor1_indeces = indeces(indeces0);
            sensor2_indeces = setdiff(indeces, sensor1_indeces);
            if (data_sorted( indeces(1), 3) == 0)
                data_sorted(sensor1_indeces,5) = 1*ones(length(sensor1_indeces),1); % camera flag
                data_sorted(sensor1_indeces,4) = block_num*ones(length(sensor1_indeces),1);
                block_num = block_num+1;
                data_sorted(sensor2_indeces,5) =2*ones(length(sensor2_indeces),1); % radar flag
                data_sorted(sensor2_indeces,4) = block_num*ones(length(sensor2_indeces),1);
                block_num = block_num+1;
            else
                data_sorted(sensor1_indeces,5) =2*ones(length(sensor1_indeces),1); % radar flag
                data_sorted(sensor1_indeces,4) = block_num*ones(length(sensor1_indeces),1);
                block_num = block_num+1;

                data_sorted(sensor2_indeces,5) =1*ones(length(sensor2_indeces),1); % camera flag
                data_sorted(sensor2_indeces,4) =block_num*ones(length(sensor2_indeces),1);
                block_num = block_num+1;
            end
        end
    else
        data_sorted(indeces_orig,5) = 4*ones(length(indeces_orig),1); % no measument recieved
        data_sorted(indeces_orig,4) = block_num*ones(length(indeces_orig),1);
        block_num = block_num+1;
    end
    if ~isempty(static_target_indices)
        data_sorted(static_target_indices,:) = [];
    end
    static_target_indices =[];
end
frame_prev = data_sorted(1,9);
data_sorted(:,10) = zeros(size(data_sorted,1),1);
block_num_prev = data_sorted(1,4);
for t = 2:size(data_sorted,1)
    current_check = data_sorted(t,9);
    block_num_curr = data_sorted(t,4);
    if (current_check <= frame_prev)
        if (block_num_curr == block_num_prev)
            frame_prev = data_sorted(t, 9);
            block_num_prev = data_sorted(t,4);
        else
            data_sorted(t, 10) = 1;
        end
    else
        frame_prev = data_sorted(t, 9);
        block_num_prev = data_sorted(t,4);
    end
end
data_sorted(12:13, :) = [];
frame_set = unique(data_sorted(:,4));

for k = 1:length(frame_set)
    indices = find(data_sorted(:,4) == frame_set(k));
    flag = data_sorted(indices(1),5); % 3 for combine, 2 for cam, 1 for radar
    if (flag == 4)
        % if no measument recieved incrment deletion and contnue, no
        % prediction (since no delta t to work with), no update nothing
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

        % 4th row lists the blocks of radar and/or camera data considered every EKF predict / update iteratio
        if flag == 3
            indeces0 = data_sorted(indices, 3) == 1;% 1 encodes for radar meas.
            radar_indices = indices(indeces0);
            camera_indeces = setdiff(indices, radar_indices);
            measurment(k).y_rad_meas = data_sorted(radar_indices,6:8);
            y_rad_mea_vect = measurment(k).y_rad_meas;
            [rows, ~] = size(y_rad_mea_vect);
            Rmat = zeros(3,3,rows);
            for q = 1:rows
                x = y_rad_mea_vect(q,1);
                y = y_rad_mea_vect(q,2);
                theta = atan2(y, x); % cleaner than atan(y/x)
                range = sqrt(x^2 + y^2);
                var_x = (cos(theta))^2*(range_res^2/4) +(range*sin(theta))^2*(azimuth_res^2/4); 
                var_y = (sin(theta))^2*(range_res^2/4) + (range*cos(theta))^2*(azimuth_res^2/4);
                doppler_res = (rvel_res^2)/4;
                Rmat(:,:,q) = [var_x 0 0; 0 var_y 0; 0 0 doppler_res];
            end
            measurment(k).y_cam_meas = data_sorted(camera_indeces,6:7);

        elseif flag == 1
            measurment(k).y_rad_meas = [];
            measurment(k).y_cam_meas = data_sorted(indices,6:7);
            Rmat =[];
        elseif flag == 2
            measurment(k).y_rad_meas = data_sorted(indices,6:8);
            y_rad_mea_vect = measurment(k).y_rad_meas;
            [rows, ~] = size(y_rad_mea_vect);
            Rmat = zeros(3,3,rows);
            for q = 1:rows
                x = y_rad_mea_vect(q,1);
                y = y_rad_mea_vect(q,2);
                theta = atan2(y, x); % cleaner than atan(y/x)
                range = sqrt(x^2 + y^2);
                var_x = (cos(theta))^2*(range_res^2/4) +(range*sin(theta))^2*(azimuth_res^2/4);
                var_y = (sin(theta))^2*(range_res^2/4) +(range*cos(theta))^2*(azimuth_res^2/4);
                doppler_res = (rvel_res^2)/4;
                Rmat(:,:,q) = [var_x 0 0; 0 var_y 0; 0 0 doppler_res];

             end
             measurment(k).y_cam_meas = [];
        end
        % this function updates the measument strust with the currently
        % considered radar and/or camera measuments for iteration k
        %%%%%%%%%%%%%%%%%%% Kalman Filter Prediction Module %%%%%%%%%%%%%%%%%%%%%%%%%%%

         frame_timestampe = data_sorted(indices(1),1);
         delta_t = abs(frame_timestampe- last_tracker_update);
         last_tracker_update = frame_timestampe;

         F = [1 0 delta_t 0;
            0 1 0 delta_t;
            0 0 1 0;
            0 0 0 1];
         if (data_sorted(indices(1),10) == 0)
             Q = [(std_ax^2)*delta_t^4/4 0 (std_ax^2)*delta_t^3/2 0;
             0 (std_ay^2)*delta_t^4/4 0 (std_ay^2)*delta_t^3/2;
             (std_ax^2)*delta_t^3/2 0 (std_ax^2)*delta_t^2 0;
             0 (std_ay^2)*delta_t^3/2 0 (std_ay^2)*delta_t^2];
             for q = 1:length(Tracks)
                if strcmp(Tracks(q).status,'active') ||strcmp(Tracks(q).status,'partially inactive')
                    Tracks(q).P_priori = [];
    
                    Tracks(q).P_priori = F*Tracks(q).P_posteriori*F'+ Q;
                    Tracks(q).states_priori = [];
                    Tracks(q).states_priori = F*(Tracks(q).states_posteriori);
                else
                    continue
                end
             end
         else
             Q = 0;
             F_retrodict = inv(F);
             for q = 1:length(Tracks)
                if strcmp(Tracks(q).status,'active') || strcmp(Tracks(q).status,'partially inactive')
                    % save P priori for other calculations
                    Pk_ = Tracks(q).P_priori;
                    Tracks(q).P_priori = F_retrodict*Pk_*F_retrodict;
                    xk_ = Tracks(q).states_priori;
                    Tracks(q).states_priori = F_retrodict*(Tracks(q).states_priori);
                else
                    continue
                end
             end
         end
         % the function computes delta_t, F, Q, and computes the P_priori matrix
         % states_prriori vector
         if ~isempty(measurment(k).y_rad_meas)
            for g = 1:length(Tracks)
                if strcmp(Tracks(g).status,'inactive')
                    continue
                else
                    x = Tracks(g).states_priori(1);y = Tracks(g).states_priori(2);
                    vx = Tracks(g).states_priori(3);vy = Tracks(g).states_priori(4);
                    r2 = x^2 + y^2; r = sqrt(r2);
                    if r < 1e-2, r = 1e-2; r2 = r^2; end
                    dvr_dx = ( y*(vx*y - vy*x) ) / (r2^(3/2));
                    dvr_dy = ( x*(-vx*y + vy*x) ) / (r2^(3/2));
                    dvr_dvx = x / r;
                    dvr_dvy = y / r;
                    Tracks(g).H = [];
        
                     Tracks(g).H = [1 0 0 0;
                                    0 1 0 0;
                                    dvr_dx dvr_dy dvr_dvx dvr_dvy];
                end
            end
         end

        Hcam = [1 0 0 0; 0 1 0 0];
        % measument prediction step as part of the target tracking logic

        for r = 1:length(Tracks)
            if strcmp(Tracks(r).status,'inactive')
                continue
            else
                if (flag == 1) % camera
                    Tracks(r).y_predicted = [];
                    Tracks(r).H = Hcam;
                    Tracks(r).y_predicted = Hcam*Tracks(r).states_priori;
                elseif (flag == 3) %combine
                    Tracks(r).y_combined_predicted =[];
                    Tracks(r).y_combined_predicted = ([Tracks(r).H;Hcam])*(Tracks(r).states_priori);
                else % radar
                    Tracks(r).y_predicted =[];
                    Tracks(r).y_predicted = (Tracks(r).H)*(Tracks(r).states_priori);
                end
            end
        end
        if (flag == 2) || (flag == 3)

            COST_PENALTY = 1e3; % large number for unassigned
            nTracks = length(Tracks);
            y_rad_vect = measurment(k).y_rad_meas;
            [nMeas, ~] = size(y_rad_vect);
            Cost_function_rad = COST_PENALTY * ones(sum(~strcmp({Tracks.status},'inactive')), nMeas );
            row1 = 0; active_tracks = [];
            for n = 1:nTracks
                if strcmp(Tracks(n).status, 'inactive')
                    continue
                end
                row1 = row1+1;
                active_tracks =[active_tracks;row1 n];
                S_rad = Tracks(n).H * Tracks(n).P_priori * Tracks(n).H';
                for o = 1:nMeas
                    S = S_rad + Rmat(:,:, o);
                    if flag ==3
                        y_pred= Tracks(n).y_combined_predicted(1:3);
                    else
                        y_pred= Tracks(n).y_predicted;
                    end
                    residual = measurment(k).y_rad_meas(o,:)' - y_pred;
                    S = (S +S')/2;
                    d2 = residual'*(S\residual);
                    %d2 = robust_mahalanobis(S, residual);
                    if d2 < G_rad
                        Cost_function_rad(row1, o) = d2;
                    end
                end
            end
        end
        if (flag == 1) || (flag == 3) % camera only flag or combine flag
             G_rad = 5.991;
             COST_PENALTY = 1e3; % large number for unassigned
             nTracks = length(Tracks);
             nMeas = size(measurment(k).y_cam_meas, 1);
             Cost_function_cam = COST_PENALTY * ones(sum(~strcmp({Tracks.status},'inactive')), nMeas );
            
             row2 = 0; active_tracks = [];
             for n = 1:nTracks
                if strcmp(Tracks(n).status, 'inactive')
                    continue
                end
                row2 = row2+1;
                active_tracks =[active_tracks;row2 n];
                S_rad = Hcam * Tracks(n).P_priori *Hcam';
                for o = 1:nMeas
                    S = S_rad + Rcam;
                    if flag ==3
                        y_pred= Tracks(n).y_combined_predicted(4:5);
                    else
                        y_pred= Tracks(n).y_predicted;
                    end
                    residual = measurment(k).y_cam_meas(o,:)' - y_pred;
                    d2 = robust_mahalanobis(S, residual);
                    if d2 < G_rad
                        Cost_function_cam(row2, o) = d2;
                    end
                end
             end
         end
         if (flag == 1)
             [~, measuments ] = size(Cost_function_cam);
             COST_PENALTY = 1e3; to_be_deleted = [];
             for a = 1:row2
                Summ = sum(Cost_function_cam(a, :)== COST_PENALTY);
                track_index = active_tracks(a,2);
            
                if (Summ == measuments)
                    if (Tracks(track_index).deletion_increment >=del_threshold)
                    Tracks(track_index).status = 'inactive';
                    else
                        Tracks(track_index).status = 'partially inactive';
                        Tracks(track_index).deletion_increment =Tracks(track_index).deletion_increment +1;
                    end
                    to_be_deleted = [to_be_deleted;a];
                else
                    Tracks(track_index).deletion_increment = 0;
                end
             end
             Cost_function_cam(to_be_deleted,:) = [];
             active_tracks(to_be_deleted,:) = [];
         elseif (flag == 3)
             [~, measuments ] = size(Cost_function_cam);
             COST_PENALTY = 1e3;
             to_be_deleted = [];
             for tao = 1:row2
                Summ = sum(Cost_function_cam(tao, :)== COST_PENALTY);
                track_index = active_tracks(tao,2);
                if (Summ == measuments)
                    if (Tracks(track_index).deletion_increment >=del_threshold)
                        Tracks(track_index).status = 'inactive';
                    else
                        Tracks(track_index).status = 'partially inactive';
                        Tracks(track_index).deletion_increment = ...
                        Tracks(track_index).deletion_increment + 1;
                    end

                    to_be_deleted = [to_be_deleted; tao];

                else
                    Tracks(track_index).deletion_increment = 0;
                end
            end

            Cost_function_cam(to_be_deleted,:) = [];
            active_tracks(to_be_deleted,:) = [];
            
            [~, measuments] = size(Cost_function_rad);
            COST_PENALTY = 1e3;
            to_be_deleted = [];
            
            for she = 1:row1
            
                Summ = sum(Cost_function_rad(she,:) == COST_PENALTY);
                track_index = active_tracks(she,2);
            
                if (Summ == measuments)
            
                    if (Tracks(track_index).deletion_increment >= del_threshold)
                        Tracks(track_index).status = 'inactive';
                    else
                        Tracks(track_index).status = 'partially inactive';
                        Tracks(track_index).deletion_increment = ...
                            Tracks(track_index).deletion_increment + 1;
                    end
            
                    to_be_deleted = [to_be_deleted; she];
            
                else
                    Tracks(track_index).deletion_increment = 0;
                end
            end
            
            Cost_function_rad(to_be_deleted,:) = [];
            active_tracks(to_be_deleted,:) = [];
            
            else
            
                [~, measuments] = size(Cost_function_rad);
                COST_PENALTY = 1e3;
                to_be_deleted = [];
            
                for tel = 1:row1
            
                    track_index = active_tracks(tel,2);
                    Summ = sum(Cost_function_rad(tel,:) == COST_PENALTY);
            
                    if (Summ == measuments)
            
                        if (Tracks(track_index).deletion_increment >= del_threshold)
                            Tracks(track_index).status = 'inactive';
                        else
                            Tracks(track_index).status = 'partially inactive';
                            Tracks(track_index).deletion_increment = ...
                                Tracks(track_index).deletion_increment + 1;
                        end
            
                        to_be_deleted = [to_be_deleted; tel];
            
                    else
                        Tracks(track_index).deletion_increment = 0;
                    end
                end
            
                Cost_function_rad(to_be_deleted,:) = [];
                active_tracks(to_be_deleted,:) = [];
            end
            
            if (flag == 3)
            
                [matching_rad, ~] = matchpairs(Cost_function_rad, 10000);
                [matching_cam, ~] = matchpairs(Cost_function_cam, 10000);
            
                if ~isempty(matching_rad)
            
                    for y = 1:length(matching_rad(:,1))
            
                        rowIdx = matching_rad(y,1);
                        ind_track = active_tracks(rowIdx,2);
            
                        meas1 = measurment(k).y_rad_meas(matching_rad(y,2),:);
            
                        if ~isempty(matching_cam)
            
                            indexxx = find(ind_track == matching_cam(:,1));
            
                            if ~isempty(indexxx)
            
                                meas2 = measurment(k).y_cam_meas(matching_cam(indexxx,2),:);
                                y_combined = [meas1(:); meas2(:)];
            
                                Tracks(ind_track).assigned_meas = [];
                                Tracks(ind_track).assigned_meas = y_combined;
                                Tracks(ind_track).sensor = 'combined';
            
                                if isempty(Tracks(ind_track).assignment_count)
                                    Tracks(ind_track).assignment_count = 1;
                                else
                                    Tracks(ind_track).assignment_count = ...
                                        Tracks(ind_track).assignment_count + 1;
                                end
            
                                Tracks(ind_track).H = ...
                                    [Tracks(ind_track).H; Hcam];
            
                                Tracks(ind_track).Rsub = ...
                                    [Rmat(:,:,matching_rad(y,2)) zeros(3,2);
                                     zeros(2,3) Rcam];
            
                                Tracks(ind_track).deletion_increment = 0;
                                Tracks(ind_track).status = 'active';
            
                            else
            
                                if (Tracks(ind_track).deletion_increment >= del_threshold)
                                    Tracks(ind_track).status = 'inactive';
                                else
                                    Tracks(ind_track).deletion_increment = ...
                                        Tracks(ind_track).deletion_increment + 1;
                                    Tracks(ind_track).status = 'partially inactive';
                                end
                            end
                        end
                    end
                end
            
            elseif (flag == 1)
            
                [matching_cam, ~] = matchpairs(Cost_function_cam, 10000);
            
                if ~isempty(matching_cam)
            
                    for y = 1:length(matching_cam(:,1))
            
                        rowIdx = matching_cam(y,1);
                        index = active_tracks(rowIdx,2);
            
                        Tracks(index).sensor = 'camera';
                        Tracks(index).assigned_meas = [];
                        Tracks(index).assigned_meas = ...
                            measurment(k).y_cam_meas(matching_cam(y,2),:);
            
                        Tracks(index).Rsub = [];
                        Tracks(index).Rsub = Rcam;
                        Tracks(index).H = [];
                        Tracks(index).H = Hcam;
            
                        if isempty(Tracks(index).assignment_count)
                            Tracks(index).assignment_count = 1;
                        else
                            Tracks(index).assignment_count = ...
                                Tracks(index).assignment_count + 1;
                        end
            
                        Tracks(index).status = 'active';
                    end
                end
            
            else
            
                [matching_rad, ~] = matchpairs(Cost_function_rad, 10000);
            
                for y = 1:length(matching_rad(:,1))
            
                    rowIdx = matching_rad(y,1);
                    index = active_tracks(rowIdx,2);
            
                    Tracks(index).sensor = 'radar';
                    Tracks(index).assigned_meas = [];
                    Tracks(index).assigned_meas = ...
                        measurment(k).y_rad_meas(matching_rad(y,2),:);
            
                    if isempty(Tracks(index).assignment_count)
                        Tracks(index).assignment_count = 1;
                    else
                        Tracks(index).assignment_count = ...
                            Tracks(index).assignment_count + 1;
                    end
            
                    Tracks(index).Rsub = Rmat(:,:,matching_rad(y,2));
                    Tracks(index).status = 'active';
                end
            end
            
            if (flag == 3)
            
                all_indices = 1:size(measurment(k).y_rad_meas,1);
                assigned_indices = matching_rad(:,2);
                unassigned_indices = setdiff(all_indices, assigned_indices);
            
                unassigned_rad = ...
                    measurment(k).y_rad_meas(unassigned_indices,:);
                R_unassigned = Rmat(:,:,unassigned_indices);
            
                if ~isempty(unassigned_rad)
            
                    curr_tracks = length(Tracks);
            
                    for J = 1:size(unassigned_rad,1)
            
                        Tracks(curr_tracks + J).sensor = 'radar';
                        Tracks(curr_tracks + J).assigned_meas = [];
                        Tracks(curr_tracks + J).assigned_meas = unassigned_rad(J,:);
                        Tracks(curr_tracks + J).assignment_count = 1;
            
                        Tracks(curr_tracks + J).Rsub = R_unassigned(:,:,J);
                        Tracks(curr_tracks + J).P_priori = [];
                        Tracks(curr_tracks + J).P_priori = INIT_COV;
            
                        vr_x = unassigned_rad(J,1);
                        vr_y = unassigned_rad(J,2);
                        vr_vx = INIT_VELOCITY;
                        vr_vy = INIT_VELOCITY;
            
                        Tracks(curr_tracks + J).H = [];
                        Tracks(curr_tracks + J).H = ...
                            [1 0 0 0;
                             0 1 0 0;
                             vr_x vr_y vr_vx vr_vy];
            
                        Tracks(curr_tracks + J).states_priori = [];
                        Tracks(curr_tracks + J).states_priori = ...
                            [vr_x vr_y vr_vx vr_vy];
            
                        Tracks(curr_tracks + J).y_predicted = [];
                        Tracks(curr_tracks + J).y_predicted = ...
                            Tracks(curr_tracks + J).H * ...
                            Tracks(curr_tracks + J).states_priori';
            
                        Tracks(curr_tracks + J).deletion_increment = 0;
                        Tracks(curr_tracks + J).status = 'active';
                    end
                end
                all_indices = 1:size(measurment(k).y_cam_meas,1);
            assigned_indices = matching_cam(:,2);
            unassigned_indices = setdiff(all_indices, assigned_indices);
            unassigned_cam = measurment(k).y_cam_meas(unassigned_indices,:);
            
            if ~isempty(unassigned_cam)
            
                curr_tracks = length(Tracks);
            
                for J = 1:size(unassigned_cam,1)
            
                    Tracks(curr_tracks + J).sensor = 'camera';
                    Tracks(curr_tracks + J).assigned_meas = [];
                    Tracks(curr_tracks + J).assigned_meas = unassigned_cam(J,:);
                    Tracks(curr_tracks + J).assignment_count = 1;
                    Tracks(curr_tracks + J).Rsub = Rcam;
                    Tracks(curr_tracks + J).P_priori = [];
                    Tracks(curr_tracks + J).P_priori = INIT_COV;
            
                    vr_x = unassigned_cam(J,1);
                    vr_y = unassigned_cam(J,2);
                    vr_vx = INIT_VELOCITY;
                    vr_vy = INIT_VELOCITY;
            
                    Tracks(curr_tracks + J).H = [];
                    Tracks(curr_tracks + J).H = Hcam;
                    Tracks(curr_tracks + J).states_priori = [];
                    Tracks(curr_tracks + J).states_priori = [vr_x vr_y vr_vx vr_vy];
                    Tracks(curr_tracks + J).y_predicted = [];
                    Tracks(curr_tracks + J).y_predicted = ...
                        Tracks(curr_tracks + J).H * ...
                        Tracks(curr_tracks + J).states_priori';
            
                    Tracks(curr_tracks + J).deletion_increment = 0;
                    Tracks(curr_tracks + J).status = 'active';
            
                end
            end
            
            elseif (flag == 1)
            
                all_indices = 1:size(measurment(k).y_cam_meas,1);
                assigned_indices = matching_cam(:,2);
                unassigned_indices = setdiff(all_indices, assigned_indices);
            
                if ~isempty(unassigned_indices)
            
                    unassigned_cam = ...
                        measurment(k).y_cam_meas(unassigned_indices,:);
                    curr_tracks = length(Tracks);
            
                    for J = 1:size(unassigned_cam,1)
            
                        Tracks(curr_tracks + J).sensor = 'camera';
                        Tracks(curr_tracks + J).assigned_meas = [];
                        Tracks(curr_tracks + J).assigned_meas = unassigned_cam(J,:);
                        Tracks(curr_tracks + J).assignment_count = 1;
                        Tracks(curr_tracks + J).Rsub = Rcam;
                        Tracks(curr_tracks + J).P_priori = [];
                        Tracks(curr_tracks + J).P_priori = INIT_COV;
            
                        vr_x = unassigned_cam(J,1);
                        vr_y = unassigned_cam(J,2);
                        vr_vx = INIT_VELOCITY;
                        vr_vy = INIT_VELOCITY;
            
                        Tracks(curr_tracks + J).H = [];
                        Tracks(curr_tracks + J).H = Hcam;
                        Tracks(curr_tracks + J).states_priori = [];
                        Tracks(curr_tracks + J).states_priori = [vr_x vr_y vr_vx vr_vy];
                        Tracks(curr_tracks + J).y_predicted = [];
                        Tracks(curr_tracks + J).y_predicted = ...
                            Tracks(curr_tracks + J).H * ...
                            Tracks(curr_tracks + J).states_priori';
            
                        Tracks(curr_tracks + J).deletion_increment = 0;
                        Tracks(curr_tracks + J).status = 'active';
            
                    end
                end
            
            else
            
                all_indices = 1:size(measurment(k).y_rad_meas,1);
                assigned_indices = matching_rad(:,2);
                unassigned_indices = setdiff(all_indices, assigned_indices);
            
                if ~isempty(unassigned_indices)
            
                    unassigned_rad = ...
                        measurment(k).y_rad_meas(unassigned_indices,:);
                    R_unassigned = Rmat(:,:,unassigned_indices);
                    curr_tracks = length(Tracks);
            
                    for J = 1:size(unassigned_rad,1)
            
                        Tracks(curr_tracks + J).sensor = 'radar';
                        Tracks(curr_tracks + J).assigned_meas = [];
                        Tracks(curr_tracks + J).assigned_meas = unassigned_rad(J,:);
                        Tracks(curr_tracks + J).assignment_count = 1;
                        Tracks(curr_tracks + J).Rsub = R_unassigned(:,:,J);
                        Tracks(curr_tracks + J).P_priori = [];
                        Tracks(curr_tracks + J).P_priori = INIT_COV;
            
                        x = unassigned_rad(J,1);
                        y = unassigned_rad(J,2);
                        vr = unassigned_rad(J,3);
            
                        vx = vr;
                        vy = vr;
            
                        r2 = x^2 + y^2;
                        r = sqrt(r2);
            
                        if r < 1e-2
                            r = 1e-2;
                            r2 = r^2;
                        end
            
                        dvr_dx = (y * (vx*y - vy*x)) / (r2^(3/2));
                        dvr_dy = (x * (-vx*y + vy*x)) / (r2^(3/2));
                        dvr_dvx = x / r;
                        dvr_dvy = y / r;
            
                        Tracks(curr_tracks + J).H = [];
                        Tracks(curr_tracks + J).H = ...
                            [1 0 0 0;
                             0 1 0 0;
                             dvr_dx dvr_dy dvr_dvx dvr_dvy];
            
                        Tracks(curr_tracks + J).states_priori = [];
                        Tracks(curr_tracks + J).states_priori = [x y vx vy];
                        Tracks(curr_tracks + J).y_predicted = [];
                        Tracks(curr_tracks + J).y_predicted = ...
                            Tracks(curr_tracks + J).H * ...
                            Tracks(curr_tracks + J).states_priori';
            
                        Tracks(curr_tracks + J).deletion_increment = 0;
                        Tracks(curr_tracks + J).status = 'active';
            
                    end
                end
            end
            
               
            
            %%%%%%%%%%%%%%%%%%% Combined KAlman Filter Update Module
            nTracks = length(Tracks);
            
            if (data_sorted(indices(1),10) == 0)
            
                for p = 1:nTracks
            
                    if strcmp(Tracks(p).status,'inactive') || ...
                       strcmp(Tracks(p).status,'partially inactive')
                        continue
                    else
            
                        if strcmp(Tracks(p).sensor,'combined')
            
                            K = (Tracks(p).P_priori * Tracks(p).H') / ...
                                (Tracks(p).H * Tracks(p).P_priori * Tracks(p).H' + ...
                                 Tracks(p).Rsub);
            
                            Tracks(p).P_posteriori = [];
                            Tracks(p).P_posteriori = ...
                                (eye(nStates) - K * Tracks(p).H) * ...
                                Tracks(p).P_priori * ...
                                (eye(nStates) - K * Tracks(p).H)' + ...
                                K * Tracks(p).Rsub * K';
            
                            Tracks(p).states_posteriori = [];
                            Tracks(p).states_priori = Tracks(p).states_priori(:);
                            Tracks(p).states_posteriori = ...
                                Tracks(p).states_priori + ...
                                K * (Tracks(p).assigned_meas - ...
                                     Tracks(p).y_combined_predicted);
            
                            state_history(k,p,:) = Tracks(p).states_posteriori(:);
            
                        elseif strcmp(Tracks(p).sensor,'radar') || ...
                               strcmp(Tracks(p).sensor,'camera')
            
                            K = Tracks(p).P_priori * Tracks(p).H' / ...
                                (Tracks(p).H * Tracks(p).P_priori * Tracks(p).H' + ...
                                 Tracks(p).Rsub);
            
                            if strcmp(Tracks(p).sensor,'radar')
                                K_rad = K;
                            else
                                K_cam = K;
                            end
            
                            Tracks(p).P_posteriori = [];
                            Tracks(p).P_posteriori = ...
                                (eye(nStates) - K * Tracks(p).H) * ...
                                Tracks(p).P_priori * ...
                                (eye(nStates) - K * Tracks(p).H)' + ...
                                K * Tracks(p).Rsub * K';
            
                            Tracks(p).states_posteriori = [];
                            Tracks(p).states_priori = Tracks(p).states_priori(:);
                            Tracks(p).states_posteriori = ...
                                Tracks(p).states_priori + ...
                                K * (Tracks(p).assigned_meas' - ...
                                     Tracks(p).y_predicted);
            
                            state_history(k,p,:) = Tracks(p).states_posteriori(:);
            
                        end
                    end
                end
            
            else
            
                for p = 1:nTracks
            
                    if strcmp(Tracks(p).status,'inactive') || ...
                       strcmp(Tracks(p).status,'partially inactive')
                        continue
                    else
            
                        if strcmp(Tracks(p).sensor,'combined')
            
                            Tracks(p).P_posteriori = ...
                                (eye(nStates) - K * Tracks(p).H) * ...
                                Tracks(p).P_priori * ...
                                (eye(nStates) - K * Tracks(p).H)' + ...
                                K * Tracks(p).Rsub * K';
            
                            Tracks(p).states_posteriori = [];
                            Tracks(p).states_priori = Tracks(p).states_priori(:);
                            Tracks(p).states_posteriori = ...
                                xk_ + K * ...
                                (Tracks(p).assigned_meas - ...
                                 Tracks(p).y_combined_predicted);
            
                            state_history(k,p,:) = Tracks(p).states_posteriori(:);
            
                        elseif strcmp(Tracks(p).sensor,'radar') || ...
                               strcmp(Tracks(p).sensor,'camera')
            
                            if strcmp(Tracks(p).sensor,'radar')
                                K = K_rad;
                            else
                                if ~isempty(K_cam)
                                    K = K_cam;
                                else
                                    Tracks(p).H = Hcam;
                                    K = Tracks(p).P_priori * Tracks(p).H' / ...
                                        (Tracks(p).H * Tracks(p).P_priori * ...
                                         Tracks(p).H' + Tracks(p).Rsub);
                                end
                            end
            
                            Tracks(p).P_posteriori = ...
                                (eye(nStates) - K * Tracks(p).H) * ...
                                Tracks(p).P_priori * ...
                                (eye(nStates) - K * Tracks(p).H)' + ...
                                K * Tracks(p).Rsub * K';
            
                            Tracks(p).states_priori = Tracks(p).states_priori(:);
                            Tracks(p).states_posteriori = ...
                                xk_ + K * ...
                                (Tracks(p).assigned_meas' - ...
                                 Tracks(p).y_predicted);
            
                            state_history(k,p,:) = Tracks(p).states_posteriori(:);
            
                        end
                    end
                end
            end
            
            Rmat = [];
            to_be_deleted = [];

    end
end

%% === Plotting code generated using the assistance of an AI model ===
%% === Parameters ===
video_filename = 'tracker_radar_with_YOLO.mp4';
trackLineWidthBase = 1.5;trackLineWidthMax = 4;
fadeWindow = 15;fps = 15;
min_assignments = 30;xyLim = [-1 6];
%% === Load Data ===
radar_data = readmatrix('radar_log.xlsx');
frames = unique(radar_data(:,2));
xCol = 6; yCol = 7; zCol = 8; vrCol = 9;
% Assuming gt_data is in workspace with columns:
% col3 = timestamp, col4 = ID, col5 = x, col6 = y
% YOLO video
yoloVid = VideoReader('yolo_output.mp4');
%% === Preprocess GT (per-ID) ===
% Extract GT columns
gt_times_all = gt_data(:,3);gt_ids_all = gt_data(:,4);
gt_x_all = gt_data(:,5);gt_y_all = gt_data(:,6);
unique_gt_ids = unique(gt_ids_all);num_gt_ids = numel(unique_gt_ids);
% Build per-id cell arrays with ABSOLUTE timestamps (do NOT normalize)
gt_id_time = cell(num_gt_ids,1);gt_id_x = cell(num_gt_ids,1);
gt_id_y = cell(num_gt_ids,1);
gt_start_times = zeros(num_gt_ids,1);
gt_end_times = zeros(num_gt_ids,1);
for i=1:num_gt_ids

 id = unique_gt_ids(i);
 mask = gt_ids_all == id;
 % sort by absolute time for safety
 [t_sorted, idxs] = sort(gt_times_all(mask));
 gt_id_time{i} = t_sorted; % keep absolute times
 gtx = gt_x_all(mask);
 gty = gt_y_all(mask);
 gt_id_x{i} = gtx(idxs);
 gt_id_y{i} = gty(idxs);
 % record start/end in same absolute time base
 if ~isempty(t_sorted)
 gt_start_times(i) = t_sorted(1);
 gt_end_times(i) = t_sorted(end);
 else
 gt_start_times(i) = Inf;
 gt_end_times(i) = -Inf;
 end
end
%% === Setup figure and axes ===
fig_width = 1500; fig_height = 1000;
fig = figure('Color','w','Position',[100 100 fig_width fig_height]);
tiledlayout(2,3,'Padding','compact','TileSpacing','compact');
axFusion = nexttile(1);axRadarXY = nexttile(2);axRadarRV = nexttile(3);
axYOLO = nexttile(4);axRMSE = nexttile(5);axGT = nexttile(6);
%% === Video Writer ===
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = fps;
open(v);
% Variables from tracker logic
num_steps = size(state_history,1);
T_final = length(Tracks);
rmse_values = nan(num_steps,1);
%% === Fix all axes initial settings ===
xlim(axFusion,xyLim); ylim(axFusion,xyLim); axis(axFusion,'equal');
xlim(axRadarXY,xyLim); ylim(axRadarXY,xyLim); axis(axRadarXY,'equal');
xlim(axRMSE,[1 num_steps]); ylim(axRMSE,[0 2]); % initial RMSE limits
%% === YOLO Frame Count & timing ===
numYoloFrames = floor(yoloVid.Duration * yoloVid.FrameRate);
yolo_time = (0:numYoloFrames-1)/yoloVid.FrameRate;
%% === Time vectors and master end time (shortest of radar, GT, YOLO) ===
radar_time = (frames - 1)/fps; % radar timestamps in seconds
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
 % after ID's last time -> use last known position
 gt_curr_pos(i,:) = [gt_id_x{i}(end), gt_id_y{i}(end)];
 end
 end
 end
 %% --- Fusion / Tracker subplot ---
 cla(axFusion); hold(axFusion,'on');
 rmse_sum = 0; rmse_count = 0;
 for p = 1:T_final
 if ~isfield(Tracks(p),'assignment_count') || Tracks(p).assignment_count <min_assignments
 continue;
 end
 % Extract states up to this radar-driven step
 xs = squeeze(state_history(1:tidx,p,1));
 ys = squeeze(state_history(1:tidx,p,2));
 validIdx = find(~isnan(xs) & ~isnan(ys) & ~(xs==0 & ys==0));
 if numel(validIdx) < 2, continue; end
 % --- Fading trail ---
 fade_len = min(fadeWindow, numel(validIdx));
 idx_seg = validIdx(end-fade_len+1:end);
 fade_alpha = linspace(0.2,1,fade_len);
 base_color = base_colors(p,:);
 for k = 1:fade_len-1
 seg_color = (1-fade_alpha(k))*[0.7 0.7 0.7] +fade_alpha(k)*base_color;
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
 dists(isnan(dists)) = []; % remove NaNs
 if ~isempty(dists)
 rmse_sum = rmse_sum + dists(1)^2; % nearest GT squared distance
 rmse_count = rmse_count + 1;
 end
 end
 end
 hold(axFusion,'off');
 xlim(axFusion, xyLim); ylim(axFusion, xyLim); axis(axFusion,'equal');
grid(axFusion,'on');
 title(axFusion,sprintf('Tracker Output [Time %.2f s (step %d/%d)]',current_time, tidx, nSteps));
 %% --- Radar plots ---
 cla(axRadarXY); cla(axRadarRV);
 hold(axRadarXY,'on'); hold(axRadarRV,'on');
 pts_idx = max(1, min(tidx, numel(frames))); % safe mapping to available frames index
 points = radar_data(radar_data(:,2)==frames(pts_idx),:);
 if ~isempty(points)
 x = points(:,xCol); y = points(:,yCol); vr = points(:,vrCol);
 rng = sqrt(x.^2 + y.^2 + points(:,zCol).^2);
 scatter(axRadarXY, x, y, 50, vr, 'filled');
 xlabel(axRadarXY,'X [m]'); ylabel(axRadarXY,'Y [m]');
grid(axRadarXY,'on');
 title(axRadarXY,sprintf('Radar Top-Down (Frame %d)',frames(pts_idx)));
 scatter(axRadarRV, rng, vr, 50, 'filled');
 xlabel(axRadarRV,'Range [m]'); ylabel(axRadarRV,'Radial Velocity [m/s]');
grid(axRadarRV,'on');
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
 title(axYOLO, sprintf('YOLO Detection (frame %d/%d)', yolo_idx,numYoloFrames));
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
 plot(axGT, xhist, yhist, '--', 'LineWidth', 1.2, 'Color',gt_colors(i,:));
 end
 end
 % Plot current point if defined (we set NaN if not started)
 if ~isnan(gt_curr_pos(i,1))
 scatter(axGT, gt_curr_pos(i,1), gt_curr_pos(i,2), 60, gt_colors(i,:),'filled');
 text(axGT, gt_curr_pos(i,1), gt_curr_pos(i,2), sprintf(' ID %d', unique_gt_ids(i)), 'VerticalAlignment','bottom');
 end
 end
 xlabel(axGT,'GT X [m]'); ylabel(axGT,'GT Y [m]');
 title(axGT,'Ground Truth Paths (by ID)');
 xlim(axGT, xyLim); ylim(axGT, xyLim); axis(axGT,'equal'); grid(axGT,'on');
 hold(axGT,'off');
 %% --- Render and save frame ---
 drawnow; % allow event processing and ensure figure updates
 frame = getframe(fig);
 writeVideo(v, frame);
end
%% --- Close video safely ---
close(v);
disp(['Video saved to ', video_filename]);
