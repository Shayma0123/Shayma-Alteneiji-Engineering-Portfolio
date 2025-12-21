function [Tracks, matching_rad, matching_cam] = CombinedMeasToTrackAssignment(Tracks,active_tracks, measurment, k,Hcam, Rcam, Cost_function_cam, Cost_function_rad, Rmat)

    [matching_rad, ~] = matchpairs(Cost_function_rad, 10000);
    [matching_cam, ~] = matchpairs(Cost_function_cam, 10000);

    if ~isempty(matching_rad)

        for y = 1:length(matching_rad(:,1))

            rowIdx = matching_rad(y,1); % row in Cost_function_rad
            ind_track = active_tracks(rowIdx,2); % actual Tracks index
    
            meas1 = measurment(k).y_rad_meas(matching_rad(y,2),:);

            if ~isempty(matching_cam)

                indexxx= find(ind_track == matching_cam(:,1));
        
                if ~isempty(indexxx) % case 1, a radar measument is matched with a cam measurment

                    meas2 = measurment(k).y_cam_meas(matching_cam(indexxx,2),:);

                    y_combined = [meas1(:); meas2(:)];
                    Tracks(ind_track).assigned_meas = [];
                    Tracks(ind_track).assigned_meas = y_combined;
                    Tracks(ind_track).sensor = 'combined';

                    if isempty(Tracks(ind_track).assignment_count)
                        Tracks(ind_track).assignment_count= 1;
                    else
                        Tracks(ind_track).assignment_count = Tracks(ind_track).assignment_count + 1;
                    end
                    Tracks(ind_track).H = [Tracks(ind_track).H; Hcam];
                    Tracks(ind_track).Rsub = [Rmat(:,:,matching_rad(y, 2)) zeros(3,2) ; zeros(2,3) Rcam];
                    Tracks(ind_track).deletion_increment = 0;
                    Tracks(ind_track).status = 'active';

                else % one measument only (cam or radar) is matched to a track
                    if (Tracks(ind_track).deletion_increment >=del_threshold)
                        Tracks(ind_track).status = 'inactive';
                    else
                        Tracks(ind_track).deletion_increment = Tracks(ind_track).deletion_increment + 1;    
     
                        Tracks(ind_track).status = 'partially inactive';
                    end  
                end
            end
        end 
    end
end

