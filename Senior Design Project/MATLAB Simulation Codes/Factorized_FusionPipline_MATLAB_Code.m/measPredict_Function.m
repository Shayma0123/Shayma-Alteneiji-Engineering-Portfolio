function [Tracks] = measPredict(Tracks,flag, Hcam)

    for r = 1:length(Tracks)

        % skip the meas prediction step for inactive tracks

        if strcmp(Tracks(r).status,'inactive')
            continue        
        else

            if (flag == 1) % camera
                Tracks(r).y_predicted = [];
                Tracks(r).H = Hcam; 

                Tracks(r).y_predicted = Hcam*Tracks(r).states_priori;


            elseif (flag == 3) %combine

                Tracks(r).y_combined_predicted =[];
                Tracks(r).y_combined_predicted =  ([Tracks(r).H; Hcam])*(Tracks(r).states_priori);

            else % radar 
                Tracks(r).y_predicted =[];
                Tracks(r).y_predicted = (Tracks(r).H)*(Tracks(r).states_priori);
            
            end
        end
    end
end
