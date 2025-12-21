
function [Cost_function,Tracks, active_tracks] = UnassignedTracksMangment(Cost_function,Tracks, row, active_tracks)

    [~, measuments ] = size(Cost_function);
    to_be_deleted = [];

    for a = 1:row
        Summ = sum(Cost_function(a, :)== 1000);
        track_index = active_tracks(a,2);
        if (Summ == measuments)

            if (Tracks(track_index).deletion_increment >=del_threshold) 
                Tracks(track_index).status = 'inactive'; 
            else 
                Tracks(track_index).status = 'partially inactive'; 
                Tracks(track_index).deletion_increment = Tracks(track_index).deletion_increment +1; 
            end 
            to_be_deleted = [to_be_deleted;a]; 
        else 
            Tracks(track_index).deletion_increment = 0; 
        end 
    end 
    Cost_function(to_be_deleted,:) = []; active_tracks(to_be_deleted,:) = [];
end

