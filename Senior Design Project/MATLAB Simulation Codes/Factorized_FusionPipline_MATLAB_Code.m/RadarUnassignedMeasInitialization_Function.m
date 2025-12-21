function [Tracks] = RadarUnassignedMeasInitialization(measurment,k, Rmat,Tracks, matching_rad, INIT_VELOCITY, INIT_COV)
    
    all_indices = 1:size(measurment(k).y_rad_meas,1);
    assigned_indices = matching_rad(:,2);
    unassigned_indices = setdiff(all_indices, assigned_indices);

    unassigned_rad = measurment(k).y_rad_meas(unassigned_indices,:);
    R_unassigned = Rmat(:,:,unassigned_indices); 

    if ~isempty(unassigned_rad) 
        
        curr_tracks = length(Tracks);

        for J = 1:size(unassigned_rad,1)

            Tracks(curr_tracks + J).sensor = 'radar';
            Tracks(curr_tracks + J).assigned_meas = [];
            Tracks(curr_tracks + J).assigned_meas = unassigned_rad(J, :);
            Tracks(curr_tracks + J).assignment_count =1;
            Tracks(curr_tracks + J).Rsub = R_unassigned(:,:,J);
            Tracks(curr_tracks + J).P_priori = [];
            Tracks(curr_tracks + J).P_priori =INIT_COV;

            vr_x = unassigned_rad(J, 1); vr_y = unassigned_rad(J, 2);
            vr_vx = INIT_VELOCITY; vr_vy = INIT_VELOCITY;

            Tracks(curr_tracks + J).H = [];
            Tracks(curr_tracks + J).H =[1 0 0 0; 0 1 0 0; vr_x vr_y vr_vx vr_vy];
            Tracks(curr_tracks + J).states_priori =[];
            Tracks(curr_tracks + J).states_priori =[vr_x vr_y vr_vx vr_vy];
            Tracks(curr_tracks + J).y_predicted = [];
            Tracks(curr_tracks + J).y_predicted =(Tracks(curr_tracks + J).H)*(Tracks(curr_tracks + J).states_priori');
            Tracks(curr_tracks + J).deletion_increment = 0;
            Tracks(curr_tracks + J).status = 'active';

        end
    end    
end

