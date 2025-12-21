function [Tracks,state_history] = EKFUpdate(Tracks,state_history, k)

    nTracks = length(Tracks);

    for p = 1:nTracks

        if strcmp(Tracks(p).status,'inactive') || strcmp(Tracks(p).status,'partially inactive')
            continue        
        else
            if strcmp(Tracks(p).sensor,'combined')

                K = (Tracks(p).P_priori * Tracks(p).H') /(Tracks(p).H * Tracks(p).P_priori * Tracks(p).H' + Tracks(p).Rsub);     
                Tracks(p).P_posteriori = [];
                Tracks(p).P_posteriori = (eye(nStates) - K*(Tracks(p).H))*Tracks(p).P_priori*(eye(nStates) - K*(Tracks(p).H))'+K*Tracks(p).Rsub*K';
                Tracks(p).states_posteriori = [];
                Tracks(p).states_priori = Tracks(p).states_priori(:);
                Tracks(p).states_posteriori = Tracks(p).states_priori + K*(Tracks(p).assigned_meas - Tracks(p).y_combined_predicted);
                    
                state_history(k, p, :) = Tracks(p).states_posteriori(:);
       
            elseif strcmp(Tracks(p).sensor,'radar') ||strcmp(Tracks(p).sensor,'camera')


                K = Tracks(p).P_priori * Tracks(p).H' /(Tracks(p).H * Tracks(p).P_priori * Tracks(p).H' + Tracks(p).Rsub);

                if strcmp(Tracks(p).sensor,'radar')
                    K_rad = K;
                else
                    K_cam = K;
                end
                
                Tracks(p).P_posteriori = [];
                Tracks(p).P_posteriori = (eye(nStates) - K*(Tracks(p).H))*Tracks(p).P_priori*(eye(nStates) - K*(Tracks(p).H))'+K*Tracks(p).Rsub*K';
                Tracks(p).states_posteriori = [];
                Tracks(p).states_priori = Tracks(p).states_priori(:);
                Tracks(p).states_posteriori = Tracks(p).states_priori + K*(Tracks(p).assigned_meas' - Tracks(p).y_predicted);
                       
                state_history(k, p, :) = Tracks(p).states_posteriori(:);
            end
        end
    end
end

