function [Cost_function_rad, active_tracks, row1] = RadCostFunc(Tracks,measurment, k, G_rad, Rmat, flag)

    nTracks = length(Tracks); 
    y_rad_vect = measurment(k).y_rad_meas;
    nMeas = size(y_rad_vect,1);
    % pre-allocates a cost function for active and partially active tracks
    % only
    Cost_function_rad = 1000 * ones( sum(~strcmp({Tracks.status},'inactive')), nMeas ); 

    row1 = 0; active_tracks = []; 
    for n = 1:nTracks 

        if strcmp(Tracks(n).status, 'inactive') 
            continue 
        end 
        row1 = row1+1; 
        % saves the cost function row index and the corresponding track
        % index
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
            % Mahalanobis distance eq
            d2 = residual'*(S\residual);    

            if d2 < G_rad 
                Cost_function_rad(row1, o) = d2; 
            end 
        end 
    end 
end

