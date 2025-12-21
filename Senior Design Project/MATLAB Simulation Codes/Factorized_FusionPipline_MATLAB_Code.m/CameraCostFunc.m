function [Cost_function_cam, active_tracks2, row2] = CameraCostFunc(Tracks,measurment, k, G_cam, Rcam, flag, Hcam)

    nTracks = length(Tracks); 
    y_rad_vect = measurment(k).y_rad_meas;
    nMeas = size(y_rad_vect,1);
    % pre-allocates a cost function for active and partially active tracks
    % only
    Cost_function_cam = 1000 * ones( sum(~strcmp({Tracks.status},'inactive')), nMeas ); 

    row2 = 0; active_tracks2 = []; 
    for n = 1:nTracks 

        if strcmp(Tracks(n).status, 'inactive') 
            continue 
        end 
        row2 = row2+1; 
        % saves the cost function row index and the corresponding track
        % index
        active_tracks2 =[active_tracks2;row2 n]; 

        S_rad = Hcam * Tracks(n).P_priori *Hcam'; 
        for o = 1:nMeas 
            S = S_rad + Rcam; 
            if flag ==3 
                y_pred= Tracks(n).y_combined_predicted(1:3); 
            else 
                y_pred= Tracks(n).y_predicted; 
            end 
            residual = measurment(k).y_rad_meas(o,:)' - y_pred; 
            S = (S +S')/2; 
            % Mahalanobis distance eq
            d2 = residual'*(S\residual);    

            if d2 < G_cam 
                Cost_function_cam(row2, o) = d2; 
            end 
        end 
    end 
end
