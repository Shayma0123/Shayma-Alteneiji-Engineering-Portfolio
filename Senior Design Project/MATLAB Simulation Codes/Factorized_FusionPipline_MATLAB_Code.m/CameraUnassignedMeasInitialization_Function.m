function [Tracks] = CameraUnassignedMeasInitialization(Tracks,measurment, matching_cam, k,INIT_VELOCITY, INIT_COV, Rcam, Hcam)

    all_indices = 1:size(measurment(k).y_cam_meas,1);
    assigned_indices = matching_cam(:,2);
    unassigned_indices = setdiff(all_indices, assigned_indices);
    unassigned_cam = measurment(k).y_cam_meas(unassigned_indices,:);

    if ~isempty(unassigned_indices)
        unassigned_cam = measurment(k).y_cam_meas(unassigned_indices,:);

        curr_tracks = length(Tracks);

        for J = 1:size(unassigned_cam,1)

            Tracks(curr_tracks + J).sensor ='camera';

            Tracks(curr_tracks + J).assigned_meas = [];
            Tracks(curr_tracks + J).assigned_meas = unassigned_cam(J, :);
            Tracks(curr_tracks + J).assignment_count =1;
            Tracks(curr_tracks + J).Rsub = Rcam;
            Tracks(curr_tracks + J).P_priori = [];
            Tracks(curr_tracks + J).P_priori =INIT_COV;
            vr_x = unassigned_cam(J, 1); vr_y = unassigned_cam(J, 2);
            vr_vx = INIT_VELOCITY; vr_vy = INIT_VELOCITY;

            Tracks(curr_tracks + J).H = [];
            Tracks(curr_tracks + J).H =Hcam;
            Tracks(curr_tracks + J).states_priori =[];
            Tracks(curr_tracks + J).states_priori =[vr_x vr_y vr_vx vr_vy];
            Tracks(curr_tracks + J).y_predicted = [];
            Tracks(curr_tracks + J).y_predicted =(Tracks(curr_tracks + J).H)*(Tracks(curr_tracks + J).states_priori');
            Tracks(curr_tracks + J).deletion_increment = 0;
            Tracks(curr_tracks + J).status = 'active';
        end
    end
end
