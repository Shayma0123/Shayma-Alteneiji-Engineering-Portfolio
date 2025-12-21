function [Tracks, matching_cam] = CamMeasToTrackAssignment(Cost_function_cam,active_tracks2, Tracks,measurment, k, Rcam, Hcam)

    [matching_cam, ~] = matchpairs(Cost_function_cam, 10000);

    if ~isempty(matching_cam)
       
        for y = 1:length(matching_cam(:, 1))

            row_ind = matching_cam(y,1); 
            index = active_tracks2(row_ind,2); 

            Tracks(index).sensor ='camera';

            Tracks(index).assigned_meas = []; 
            Tracks(index).assigned_meas  = measurment(k).y_cam_meas(matching_cam(y, 2),:);
            Tracks(index).Rsub = [];
            Tracks(index).Rsub = Rcam;
            Tracks(index).H = [];
            Tracks(index).H = Hcam; 

            if isempty(Tracks(index).assignment_count)
                Tracks(index).assignment_count= 1;
            else
                Tracks(index).assignment_count = Tracks(index).assignment_count + 1;          end

            Tracks(index).status = 'active';

        end

        if ~isempty(Cost_function_cam)
            if (size(Cost_function_cam,1)> length(Cost_function_cam(1,:)))

                assigned_tracks = active_tracks(matching_cam(:,1),2);
                unassigned_tracks = setdiff(active_tracks,assigned_tracks);

                for b = 1:length(unassigned_tracks)

                    if (Tracks(unassigned_tracks(b)).deletion_increment >=del_threshold)
                        Tracks(unassigned_tracks(b)).status = 'inactive';
                    else
                        Tracks(unassigned_tracks(b)).deletion_increment = Tracks(unassigned_tracks(b)).deletion_increment + 1;    
     
                        Tracks(unassigned_tracks(b)).status = 'partially inactive';
                    end  
                end
            end
        end
    end
end 
