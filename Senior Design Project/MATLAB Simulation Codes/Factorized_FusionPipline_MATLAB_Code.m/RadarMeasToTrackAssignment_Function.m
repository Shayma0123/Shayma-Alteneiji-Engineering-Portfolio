function [Tracks, matching_rad] = RadarMeasToTrackAssignment(Cost_function_rad, Tracks,Rmat, k, measurment, active_tracks)

   [matching_rad, ~] = matchpairs(Cost_function_rad, 10000); 

   for y = 1:length(matching_rad(:, 1))

       row_ind = matching_rad(y,1); 
       index = active_tracks(row_ind,2);
       Tracks(index).sensor ='radar'; 
       Tracks(index).assigned_meas = []; 
       Tracks(index).assigned_meas = measurment(k).y_rad_meas(matching_rad(y, 2),:); 
       if isempty(Tracks(index).assignment_count)
            Tracks(index).assignment_count= 1;
       else
            Tracks(index).assignment_count = Tracks(index).assignment_count + 1;
       end
       Tracks(index).Rsub = Rmat(:,:,matching_rad(y, 2)); 
       Tracks(index).status = 'active'; 
   end

   if ~isempty(Cost_function_rad)
       if (size(Cost_function_rad,1)> length(Cost_function_rad(1,:)))
           assigned_tracks = active_tracks(matching_rad(:,1),2);
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
            
