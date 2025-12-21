function [data_sorted] = sensor_data_process(camerasynthetic,radarsynthetic)

    t_cam_meas_frame = camerasynthetic(:,2); % meas frame timestampe
    cam_pi_clock = camerasynthetic(:,3); % timestampe corresponding to when the Rassberry Pi recieves the measurment, after the YOLO procassing 
    cam_meas = camerasynthetic(:,10:11); % BBoxes centroids of the YOLO detections
    
    t_rad_meas_frame = radarsynthetic(:,3); % meas frame timestampe
    rad_meas = radarsynthetic(:,[6:7, 9]); % radar x, y, and vr measurments 
    rad_pi_clock = radarsynthetic(:,4); % timestampe corresponding to when the Rassberry Pi recieves the measurment, after the data parsing step
    rad_meas(:,1:2) = rad_meas(:,1:2);
    
    % to produce discrete frames timestamps 
    FPS = 1000; 
    radar_dt = 1/FPS;
    camera_dt = 1/FPS;
    t_rad_meas_frame = round(t_rad_meas_frame/radar_dt)*radar_dt;
    t_cam_meas_frame = round(t_cam_meas_frame/camera_dt)*camera_dt;
    
    
    % row 1 = meas frame, row 2 = frame meas timestamp after data procassing,
    % row 3 = sensor flag (0 for camera, 1 for radar), row 4 = procassed blocks num, row 5
    % = EKF update flag, row 6 = x meas, row 7 = y meas, row 8 = vr meas
    data = [ t_rad_meas_frame rad_pi_clock ones(length(t_rad_meas_frame),1) zeros(length(t_rad_meas_frame),1) zeros(length(t_rad_meas_frame),1) rad_meas; 
        t_cam_meas_frame cam_pi_clock zeros(length(t_cam_meas_frame),1) zeros(length(t_cam_meas_frame),1) zeros(length(t_cam_meas_frame),1) x_y zeros(length(t_cam_meas_frame), 1)];
    
    data_sorted = sortrows(data, 2);
    
    block_num = 1;
  
    frame_groups = findgroups(data_sorted(:,1));
    
    % frame group column is 9, frame ID
    
    data_sorted(:,9) = frame_groups;
    % total number of unique frames stored for a given scene
    set = unique(frame_groups);   
    static_target_indices =[];
    
    for w = 1:length(set)
    
        % determines the index corresponding to the first measurment data in
        % the data block considered for a given frame 
        indeces_orig = find(data_sorted(:, 4) ==set(w));
    
        % checks if any of the measurments consdired is static (static meas
        % were replaced by nan at an earlier step)
        % the vector is all ones if all measurments are static
        check_static_meas = isnan(data_sorted(indeces_orig, 6));
    
        % num of meas recieved from camera or radar in a given frame
        num_recived_meas_per_fram = length(check_static_meas);
        
        % number of static measurments in that frame
        intermediate = sum(check_static_meas(:) == 1);
   
        % if the measurments recieved are not all static 
    
        if ~(intermediate == num_recived_meas_per_fram)
    
            % extracts the moving_target_indicces
    
            indeces = indeces_orig(~check_static_meas);
    
            static_target_indices = indeces_orig(check_static_meas);
    
            % for a given frame, we record the earliest time (min) a measurment was
            % 'recieved' by the rassberry pi for that frame, and the latest time (max)    
            max_pi_timestamp = max(data_sorted(indeces,2));
    
            min_pi_timestamp = min(data_sorted(indeces,2));

            % if the difference between arrival of the meas to the
            % Rassberry Pi is less that 0.05, and the measurments is of two
            % different sensors, the EKF update flag is 3, and the sensors
            % data are considered in one block ( updates data_sorted 4th row)
    
            if (max_pi_timestamp- min_pi_timestamp <0.05) && (length(unique(data_sorted(indeces,3)))>1)
    
                data_sorted(indeces,4) = block_num*ones(length(indeces),1);
                block_num = block_num+1;
                data_sorted(indeces,5) = 3*ones(length(indeces),1); % combine flag

            % else, if the difference between arrival of the meas to the
            % Rassberry Pi is less that 0.05, and the measurments is of one
            % sensor, the EKF update flag is either 1 or 2      
   
            elseif (max_pi_timestamp- min_pi_timestamp <0.05) && (length(unique(data_sorted(indeces,3)))<2)
    
                data_sorted(indeces,4) = block_num*ones(length(indeces),1);
                block_num = block_num+1;

                % the sensor flag is used to determine the EKF Update flag 
    
                if data_sorted(indeces(1),3) == 0
                    data_sorted(indeces,5) = 1*ones(length(indeces),1); % camera flag
                else 
                    data_sorted(indeces,5) = 2*ones(length(indeces),1); % radar flag
                end
                % if the difference between arrival of the meas to the
                % Rassberry Pi is not less that 0.05, and the measurments is of two
                % different sensors. This is when one sensor
                % provides data at a faster rate, while the other is
                % slower, leadeing to OOSM. OOSM measurment are flaged
                % after this step, here, block num of grouped data
                % incrments and the EKF update flag is determined based on
                % sensor type flag
    
            elseif ~(max_pi_timestamp- min_pi_timestamp <0.05) && (length(unique(data_sorted(indeces,3)))>1)
    
                % uses the sensor type flag to determine each sensor's meas indices 
                indeces0 = data_sorted(indeces, 3) == data_sorted(indeces(1), 3); 
                sensor1_indeces = indeces(indeces0);
    
                sensor2_indeces = setdiff(indeces, sensor1_indeces);
    
                if (data_sorted( indeces(1), 3) == 0)
    
                    data_sorted(sensor1_indeces,5) = 1*ones(length(sensor1_indeces),1); % camera flag
                    data_sorted(sensor1_indeces,4) = block_num*ones(length(sensor1_indeces),1); 
                    block_num = block_num+1;
    
                    data_sorted(sensor2_indeces,5) = 2*ones(length(sensor2_indeces),1); % radar flag
                    data_sorted(sensor2_indeces,4) = block_num*ones(length(sensor2_indeces),1); 
                    block_num = block_num+1;
    
                else
                    data_sorted(sensor1_indeces,5) = 2*ones(length(sensor1_indeces),1); % radar flag
                    data_sorted(sensor1_indeces,4) = block_num*ones(length(sensor1_indeces),1); 
                    block_num = block_num+1;
    
                    data_sorted(sensor2_indeces,5) = 1*ones(length(sensor2_indeces),1); % camera flag
                    data_sorted(sensor2_indeces,4) = block_num*ones(length(sensor2_indeces),1); 
                    block_num = block_num+1;
                end
            end
        else
            % if the measument recived is static radar meas, the flag is
            % set as 4
            data_sorted(indeces_orig,5) = 4*ones(length(indeces_orig),1); % no measument recieved 
            data_sorted(indeces_orig,4) = block_num*ones(length(indeces_orig),1);
            block_num = block_num+1;
    
        end

        if ~isempty(static_target_indices)
            data_sorted(static_target_indices,:) = [];
        end
        static_target_indices =[];
    end
    
    % OOSM flagging , row 10 
    frame_prev = data_sorted(1,9);
    data_sorted(:,10) = zeros(size(data_sorted,1),1);
    block_num_prev =  data_sorted(1,4);

    % when the frame of the meas previous to the currently checked frame is
    % greater than or equal to it, the meas is potentially an OOSM, if the
    % the meas does not have the same grouped meas block num, then the
    % signal is an OOSM. frame_prev and block_num_prev are ot updated 

    for t = 2:size(data_sorted,1)
        current_check = data_sorted(t,9);

        block_num_curr =  data_sorted(t,4);

        if (current_check <= frame_prev)

            if (block_num_curr == block_num_prev)
                frame_prev = data_sorted(t, 9);
                block_num_prev =  data_sorted(t,4); 
            else 
                data_sorted(t, 10) = 1;
            end
        else
            frame_prev = data_sorted(t, 9);
            block_num_prev =  data_sorted(t,4);
        end
    end
end 
