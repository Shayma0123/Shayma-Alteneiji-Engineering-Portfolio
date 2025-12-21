function [state_history,Tracks] = OOSM_EKF_Update(Tracks, Hcam, state_history, xk_ , k)
    for p = 1:nTracks

        if strcmp(Tracks(p).status,'inactive') || strcmp(Tracks(p).status,'partially inactive')
            continue        
        else

            if strcmp(Tracks(p).sensor,'combined')

                K = Tracks(p).P_priori * Tracks(p).H' /(Tracks(p).H * Tracks(p).P_priori * Tracks(p).H' + Tracks(p).Rsub);
                Tracks(p).P_posteriori = (eye(nStates) - K*(Tracks(p).H))*Tracks(p).P_priori*(eye(nStates) - K*(Tracks(p).H))'+K*Tracks(p).Rsub*K';
                Tracks(p).states_posteriori = [];
                Tracks(p).states_priori = Tracks(p).states_priori(:);
                Tracks(p).states_posteriori = xk_ + K*(Tracks(p).assigned_meas - Tracks(p).y_combined_predicted);
                               
                state_history(k, p, :) = Tracks(p).states_posteriori(:);
       
            elseif strcmp(Tracks(p).sensor,'radar') ||strcmp(Tracks(p).sensor,'camera')

                if strcmp(Tracks(p).sensor,'radar')
                    K = K_rad;

                else
                    if ~isempty(K_cam)
                        
                        K= K_cam;
                    else
                        Tracks(p).H = Hcam;
                        K = Tracks(p).P_priori * Tracks(p).H' /(Tracks(p).H * Tracks(p).P_priori * Tracks(p).H' + Tracks(p).Rsub);

                    end
                      
                end    
                    
                Tracks(p).P_posteriori = (eye(nStates) - K*(Tracks(p).H))*Tracks(p).P_priori*(eye(nStates) - K*(Tracks(p).H))'+ K*Tracks(p).Rsub*K';
                Tracks(p).states_priori = Tracks(p).states_priori(:);
                Tracks(p).states_posteriori = xk_ + K*(Tracks(p).assigned_meas' - Tracks(p).y_predicted);
                                
                state_history(k, p, :) = Tracks(p).states_posteriori(:);
            end
        end
    end
end


