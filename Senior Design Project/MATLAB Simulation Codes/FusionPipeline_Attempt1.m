
%constants
range_res = 0.293; %(m)
rvel_res = 0.31;%(m/s)
azimuth_res = pi/6; %(rad)

%%%%%%%%%%%%%%%%%%% Intilization %%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_cam_meas_frame = camerasynthetic(:,1);
cam_pi_clock = camerasynthetic(:,2);
cam_meas = camerasynthetic(:,3:4);

t_rad_meas_frame = radarsynthetic(:,1);
rad_pi_clock = radarsynthetic(:,2);
rad_meas = radarsynthetic(:,3:5);

combine = 0; stale = 0; camera_alone = 0;radar_alone = 0; 
last_tracker_update = 0; rad(1).R =[];

% Kalman Filter Initilization 
T = 1;

% Tracks structure containing the states_posteriori, states_priori,
% P_posteriori , 


nStates = 4; % number of states
Tracks(1).states_posteriori = [20; 50; 1; 1];  % column vector
Tracks(1).P_posteriori = 5*eye(nStates);   
it =0;
%%%%%%%%%%%%%%%%%%%%%%% Main Structure %%%%%%%%%%%%%%%%%%%%
state_history = zeros(10, T, nStates);
 

while (~isempty(rad_meas))
    it = it+1;
    % Are the sensors seeing the same frame AND arrive approximatly at the same
    % time 
    if ((t_cam_meas_frame(1) == t_rad_meas_frame(1) && (abs(cam_pi_clock(1) -rad_pi_clock(1)) <=0.05)))

        combine = 1;
        fprintf("camera and radar arivve at t = %2.4f",t_cam_meas_frame(1))
    else
        % which measurment arrived first?

        if (cam_pi_clock(1)< rad_pi_clock(1))

            if (t_cam_meas_frame(1) > last_tracker_update)

                camera_alone = 1;
                fprintf("camera arivve first at t = %2.4f",t_cam_meas_frame(1))
            else
                 stale = 1;
            end
        else

            if (t_rad_meas_frame(1) > last_tracker_update)

                radar_alone = 1;
                
                fprintf("radar arivve first at t = %2.4f",t_rad_meas_frame(1))
            else
                stale = 1;
            end
        end       
    end


%%%%%%%%%%%%%%%%%%% Kalman Filter Prediction Module %%%%%%%%%%%%%%%%%%%%%%%%%%%
 

    % Calculate delta t 
    delta_t = abs(t_rad_meas_frame(1,1) - rad_pi_clock(1,1));

    F = [1 0 delta_t  0;
        0 1 0 delta_t;
        0 0 1 0; 
        0 0 0 1];

    std_ax = sqrt(0.1); std_ay = sqrt(0.1);
    Q = [(std_ax^2)*delta_t^4/4 0 (std_ax^2)*delta_t^3/2 0;
        0 (std_ay^2)*delta_t^4/4 0 (std_ay^2)*delta_t^3/2;
        (std_ax^2)*delta_t^3/2 0 (std_ax^2)*delta_t^2 0;
        0 (std_ay^2)*delta_t^3/2 0 (std_ay^2)*delta_t^2];


    for q = 1:T


        Tracks(q).P_priori = [];
        Tracks(q).P_priori = F*Tracks(q).P_posteriori*F'+ Q;
        Tracks(q).states_priori = [];
        Tracks(q).states_priori  = F*(Tracks(q).states_posteriori);

    end

    if (camera_alone == 0)


        for g = 1:T

            vr_x = Tracks(g).states_priori(2)*((Tracks(g).states_priori(2)*Tracks(g).states_priori(3) - Tracks(g).states_priori(1)*Tracks(g).states_priori(4))/((Tracks(g).states_priori(1)^2 + Tracks(g).states_priori(2)^2)^(3/2)));
            vr_y = Tracks(g).states_priori(1)*((Tracks(g).states_priori(1)*Tracks(g).states_priori(4) - Tracks(g).states_priori(2)*Tracks(g).states_priori(3))/((Tracks(g).states_priori(1)^2 + Tracks(g).states_priori(2)^2)^(3/2)));
            vr_vx = Tracks(g).states_priori(1)/((Tracks(g).states_priori(1)^2 + Tracks(g).states_priori(2)^2)^(1/2));
            vr_vy = Tracks(g).states_priori(2)/((Tracks(g).states_priori(1)^2 + Tracks(g).states_priori(2)^2)^(1/2));

            Tracks(g).H = [];
            Tracks(g).H =[1 0 0 0; 0 1 0 0; vr_x vr_y vr_vx vr_vy];
        end

            indices = find(t_rad_meas_frame == t_rad_meas_frame(1));
   
  
            y_meas_rad = rad_meas(indices,:);
                        for p = 1:length(y_meas_rad)

                x = y_meas_rad(p,1); y = y_meas_rad(p,2); vr = y_meas_rad(p,3);

                if (x>0)
                    theta = atan(y/x);
                else
                    theta = atan(y/x)+pi;
                end

                range = sqrt(x^2+y^2);

                var_x = ((cos(theta))^2)*((range_res^2)/4) +((range*sin(theta))^2)*((azimuth_res^2)/4);
                var_y = ((sin(theta))^2)*((range_res^2)/4) + ((range*cos(theta))^2)*((azimuth_res^2)/4);
                doppler_res = (rvel_res^2)/4;

                rad(p).R = [var_x 0 0;
                            0 var_y 0; 
                            0 0 doppler_res];
                        end
             rad_meas(indices,:) = [];

            t_rad_meas_frame(indices,:) = [];
            rad_pi_clock(indices,:) = [];

            if (combine ==1)
                indices2 = find(t_cam_meas_frame == t_cam_meas_frame(1));
                y_meas_cam = cam_meas(indices2,:);
                cam_meas(indices2,:) = [];
                t_cam_meas_frame(indices2,:) = [];
                cam_pi_clock(indices2,:) = [];

                Rcam = [5 0; 0 5];
            end

    else 
        indices = find(t_cam_meas_frame == t_cam_meas_frame(1));
        y_meas_cam = cam_meas(indices,:);
        y_meas_rad = 0;
        cam_meas(indices,:) = [];
        t_cam_meas_frame(indices,:) = [];
        cam_pi_clock(indices,:) = []; 
        Rcam = [5 0 ; 0 5];
    end




Hcam = [1 0 0 0; 0 1 0 0];
    for Q = 1:T
        if (camera_alone == 0)

            Tracks(Q).y_rad_predicted = []; 
            Tracks(Q).y_rad_predicted = (Tracks(Q).H)*(Tracks(Q).states_priori);


            if (combine == 1)

                Tracks(Q).y_cam_predicted =[];
                Tracks(Q).y_cam_predicted = Hcam*Tracks(Q).states_priori;
                Tracks(Q).y_combined_predicted =  ([Tracks(Q).H; Hcam])*(Tracks(Q).states_priori);

            end 

        else 
            Tracks(Q).y_cam_predicted =[];
            Tracks(Q).y_cam_predicted = Hcam*Tracks(Q).states_priori;
            
        end
    end

 if (camera_alone == 0)

    G_rad = 7.815; 
    [m, ~] = size(y_meas_rad);
    Cost_function_rad = zeros(T,m);


    for n = 1:T

        Tracks(n).S_rad = [];
        Tracks(n).S_rad = Tracks(n).H*Tracks(n).P_priori*Tracks(n).H';


        for o = 1:m 

            S =  Tracks(n).S_rad + rad(o).R;

            mahalanobis_dist = (y_meas_rad(o, :)' - Tracks(n).y_rad_predicted)'*inv(S)*(y_meas_rad(o, :)' - Tracks(n).y_rad_predicted);
            if (mahalanobis_dist < G_rad)

                Cost_function_rad(n,o) = mahalanobis_dist;
            else
                Cost_function_rad(n,o) = 1000;
            end
        end 
    end
    if (combine == 1)

        G_cam = 5.991; 
        [m, ~] = size(y_meas_cam);
        Cost_function_cam = zeros(T,m);

        for n = 1:T

            Tracks(n).S_cam = Hcam*Tracks(n).P_priori*Hcam' +Rcam;

            [m, ~] = size(y_meas_cam);

            for o = 1:m 

                mahalanobis_dist = (y_meas_cam(o, :)' - Tracks(n).y_cam_predicted)'*inv(Tracks(n).S_cam)*(y_meas_cam(o,:)' - Tracks(n).y_cam_predicted);
                if (mahalanobis_dist < G_cam)

                    Cost_function_cam(n,o) = mahalanobis_dist;
                else
                    Cost_function_cam(n,o) = 1000;
                end
            end 
        end

    end
else

        G_cam = 5.991; 

        [m, ~] = size(y_meas_cam);
        Cost_function_cam = zeros(T,m);



        for n = 1:T

            Tracks(n).S_cam = Hcam*Tracks(n).P_priori*Hcam' +Rcam;

            m = length(y_meas_cam);

            for o = 1:m 

                mahalanobis_dist = (y_meas_cam(o, :)' - Tracks(n).y_cam_predicted)'*inv(Tracks(n).S_cam)*(y_meas_cam(o,:)' - Tracks(n).y_cam_predicted);
                if (mahalanobis_dist < G_cam)

                    Cost_function_cam(n,o) = mahalanobis_dist;
                else
                    Cost_function_cam(n,o) = 1000;
                end
            end 
        end
 end


if  (camera_alone == 0)

    to_be_deleted = [];

    [target, measuments ] = size(Cost_function_rad);
    for k = 1:target
        Summ = sum(find(Cost_function_rad(target, :)== 1000));
        if (Summ == measuments)
            to_be_deleted = [to_be_deleted;k];
        else
            Tracks(k).deletion_increment = 0;
        end
    end

    u = to_be_deleted;
    for k = 1:length(u)

        h =to_be_deleted(k ); 


        Tracks(h).deletion_increment = Tracks(h).deletion_increment + 1;
        if (Tracks(h).deletion_increment >=3)
            Tracks(h) = [];
            Cost_function_rad(h,:) = [];
        end
    end
    if (combine == 1)

    to_be_deleted = [];

    [target, measuments ] = size(Cost_function_cam);
    for k = 1:target
        Summ = sum(find(Cost_function_cam(target, :)== 1000));
        if (Summ == measuments)
            to_be_deleted = [to_be_deleted;k];
        else
            Tracks(k).deletion_increment = 0;
        end
    end

    u = to_be_deleted;
    for k = 1:length(u)

        h =to_be_deleted(k ); 


        Tracks(h).deletion_increment = Tracks(h).deletion_increment + 1;
        if (Tracks(h).deletion_increment >=3)
            Tracks(h) = [];
            Cost_function_cam(h,:) = [];
        end
    end

    end
else 
        to_be_deleted = [];

    [target, measuments ] = size(Cost_function_cam);
    for k = 1:target
        Summ = sum(find(Cost_function_cam(target, :)== 1000));
        if (Summ == measuments)
            to_be_deleted = [to_be_deleted;k];
        else
            Tracks(k).deletion_increment = 0;
        end
    end

    u = to_be_deleted;
    for k = 1:length(u)

        h =to_be_deleted(k ); 


        Tracks(h).deletion_increment = Tracks(h).deletion_increment + 1;
        if (Tracks(h).deletion_increment >=3)
            Tracks(h) = [];
            Cost_function_cam(h,:) = [];
        end
    end

end

   
% Hungarian Algorithm and Assignment Problem 
if (combine == 1)


    [matching_rad, ~] = matchpairs(Cost_function_rad, 10000);
    [matching_cam, ~] = matchpairs(Cost_function_cam, 10000);

    for k = 1:length(matching_rad(:,1))

        tra = matching_rad(k,1);

        meas1 = y_meas_rad(matching_rad(k,2),:);
        indexxx= find(tra == matching_cam(:,1));
        meas2 = y_meas_cam(indexxx,:);
        y_combined = [meas1(:); meas2(:)];
        Tracks(indexxx).assigned_meas = zeros(1,5);
        Tracks(indexxx).assigned_meas = y_combined;
        
        Tracks(indexxx).H = [Tracks(indexxx).H; Hcam];

        Tracks(indexxx).R = [rad(matching_rad(k, 2)).R zeros(3,2) ; zeros(2,3) Rcam];

    end 

elseif (camera_alone == 1)

    [matching_cam, ~] = matchpairs(Cost_function_cam, 10000);
    

    for k = 1:length(matching_cam(:, 1))

        index = matching_cam(k, 1);

        Tracks(index).assigned_meas = zeros(1,2); 
        Tracks(index).assigned_meas  = y_meas_cam(matching_cam(k, 2),:);
        Tracks(index).R = [];
        Tracks(index).R = Rcam;
        Tracks(index).H = [];
        Tracks(index).H = Hcam;

       
    end



else 

    [matching_rad, cost_rad] = matchpairs(Cost_function_rad, 10000);
    

    for k = 1:length(matching_rad(:, 1))

        index = matching_rad(k, 1);

        Tracks(index).assigned_meas = zeros(1,3); 
        Tracks(index).assigned_meas = y_meas_rad(matching_rad(k, 2),:);
        Tracks(index).R = rad(matching_rad(k, 2)).R;
    end

end

if (combine == 1)

    [row, colu] = size(y_meas_rad);
    [row1, colu1] = size(y_meas_cam);

    T_new1 = row - length(matching_rad(:, 1));
    T_new2 = row1 - length(matching_cam(:, 1));

    T = T +T_new1 +T_new2;

    curr_tracks_old = length(Tracks);
    curr_tracks = curr_tracks_old;

    y_meas_rad(matching_rad(:, 2),:)= [];

    rad(matching_rad(:, 2)) = [];
    
    for J = 1:T_new1

        Tracks(curr_tracks + J).assigned_meas = [];
        Tracks(curr_tracks + J).assigned_meas = y_meas_rad(J, :);
        Tracks(curr_tracks + J).R = rad(J).R;
        Tracks(curr_tracks + J).P_priori = [];
        Tracks(curr_tracks + J).P_priori =5*eye(4);

        vr_x = y_meas_rad(J, 1);
        vr_y = y_meas_rad(J, 2);
        vr_vx = 0.1;
        vr_vy = 0.1;

        Tracks(curr_tracks + J).H = [];
        Tracks(curr_tracks + J).H =[1 0 0 0; 0 1 0 0; vr_x vr_y vr_vx vr_vy];
        Tracks(curr_tracks + J).states_priori =[];
        Tracks(curr_tracks + J).states_priori =[vr_x vr_y vr_vx vr_vy];
        Tracks(curr_tracks + J).y_rad_predicted = [];
        Tracks(curr_tracks + J).y_rad_predicted =(Tracks(curr_tracks + J).H)*(Tracks(curr_tracks + J).states_priori');

    end

    curr_tracks = length(Tracks);

    y_meas_cam(matching_cam(:, 2),:)= [];


    for J = 1:T_new2
    


        Tracks(curr_tracks + J).assigned_meas = [];
        Tracks(curr_tracks + J).assigned_meas = y_meas_cam(J, :);
        Tracks(curr_tracks + J).R = Rcam;
        Tracks(curr_tracks + J).P_priori = [];
        Tracks(curr_tracks + J).P_priori =5*eye(4);

        vr_x = y_meas_cam(J, 1);
        vr_y = y_meas_cam(J, 2);
        vr_vx = 0.1;
        vr_vy = 0.1;

        Tracks(curr_tracks + J).H = [];
        Tracks(curr_tracks + J).H =Hcam;
        Tracks(curr_tracks + J).states_priori =[];
        Tracks(curr_tracks + J).states_priori =[vr_x vr_y vr_vx vr_vy];
        Tracks(curr_tracks + J).y_cam_predicted = [];
        Tracks(curr_tracks + J).y_cam_predicted =(Tracks(curr_tracks + J).H)*(Tracks(curr_tracks + J).states_priori');



    end

elseif (camera_alone == 1)

    T_new = row - length(matching_cam(:, 1));

    T = T +T_new;
    curr_tracks = length(Tracks);
    y_meas_cam(matching_cam(:, 2),:)= [];

    
    for J = 1:T_new

        Tracks(curr_tracks + J).assigned_meas = [];
        Tracks(curr_tracks + J).assigned_meas = y_meas_cam(J, :);
        Tracks(curr_tracks + J).R = Rcam;
        Tracks(curr_tracks + J).P_priori = [];
        Tracks(curr_tracks + J).P_priori =5*eye(4);

        vr_x = y_meas_cam(J, 1);
        vr_y = y_meas_cam(J, 2);
        vr_vx = 0.1;
        vr_vy = 0.1;

        Tracks(curr_tracks + J).H = [];
        Tracks(curr_tracks + J).H =Hcam;
        Tracks(curr_tracks + J).states_priori =[];
        Tracks(curr_tracks + J).states_priori =[vr_x vr_y vr_vx vr_vy];
        Tracks(curr_tracks + J).y_rad_predicted = [];
        Tracks(curr_tracks + J).y_rad_predicted =(Tracks(curr_tracks + J).H)*(Tracks(curr_tracks + J).states_priori');

    end

else 

    T_new = row - length(matching_rad(:, 1));

    T = T +T_new;
    curr_tracks = length(Tracks);
    y_meas_rad(matching_rad(:, 2),:)= [];
    rad(matching_rad(:, 2)) = [];

    
    for J = 1:T_new

        Tracks(curr_tracks + J).assigned_meas = [];
        Tracks(curr_tracks + J).assigned_meas = y_meas_rad(J, :);
        Tracks(curr_tracks + J).R = rad(J).R;
        Tracks(curr_tracks + J).P_priori = [];
        Tracks(curr_tracks + J).P_priori =5*eye(4);

        vr_x = y_meas_rad(J, 1);
        vr_y = y_meas_rad(J, 2);
        vr_vx = 0.1;
        vr_vy = 0.1;

        Tracks(curr_tracks + J).H = [];
        Tracks(curr_tracks + J).H =[1 0 0 0; 0 1 0 0; vr_x vr_y vr_vx vr_vy];
        Tracks(curr_tracks + J).states_priori =[];
        Tracks(curr_tracks + J).states_priori =[vr_x vr_y vr_vx vr_vy];
        Tracks(curr_tracks + J).y_rad_predicted = [];
        Tracks(curr_tracks + J).y_rad_predicted =(Tracks(curr_tracks + J).H)*(Tracks(curr_tracks + J).states_priori');

    end

end


% Track Merging  - will be added later 


%%%%%%%%%%%%%%%%%%% Combined KAlman Filter Update Module %%%%%%%%%%%%%%%%%%%%%%%%%%%
if (combine == 1)

    for p = 1:curr_tracks_old

       K = Tracks(p).P_priori*((Tracks(p).H)')*inv((Tracks(p).H)*(Tracks(p).P_priori)*(Tracks(p).H)' +Tracks(p).R);
       Tracks(p).P_posteriori = [];
       Tracks(p).P_posteriori = (eye(nStates) - K*(Tracks(p).H))*Tracks(p).P_priori*(eye(nStates) - K*(Tracks(p).H))'+K*Tracks(p).R*K';
       Tracks(p).states_posteriori = [];
       Tracks(p).states_priori = Tracks(p).states_priori(:);
       Tracks(p).states_posteriori = Tracks(p).states_priori + K*(Tracks(p).assigned_meas - Tracks(p).y_combined_predicted);
       state_history(it, p, :) = Tracks(p).states_posteriori(:);
    end

    for p = curr_tracks_old+1:T_new1+curr_tracks_old

       K = Tracks(p).P_priori*((Tracks(p).H)')*inv((Tracks(p).H)*(Tracks(p).P_priori)*(Tracks(p).H)' +Tracks(p).R);
       Tracks(p).P_posteriori = [];
       Tracks(p).P_posteriori = (eye(nStates) - K*(Tracks(p).H))*Tracks(p).P_priori*(eye(nStates) - K*(Tracks(p).H))'+K*Tracks(p).R*K';
       Tracks(p).states_posteriori = [];
       Tracks(p).states_priori = Tracks(p).states_priori(:);
       Tracks(p).states_posteriori = Tracks(p).states_priori + K*(Tracks(p).assigned_meas' - Tracks(p).y_rad_predicted);
       state_history(it, p, :) = Tracks(p).states_posteriori(:);
    end

    for p = T_new1+curr_tracks_old+1:T_new2+T_new1+curr_tracks_old

       K = Tracks(p).P_priori*((Tracks(p).H)')*inv((Tracks(p).H)*(Tracks(p).P_priori)*(Tracks(p).H)' +Tracks(p).R);
       Tracks(p).P_posteriori = [];
       Tracks(p).P_posteriori = (eye(nStates) - K*(Tracks(p).H))*Tracks(p).P_priori*(eye(nStates) - K*(Tracks(p).H))'+K*Tracks(p).R*K';
       Tracks(p).states_posteriori = [];
       Tracks(p).states_priori = Tracks(p).states_priori(:);
       Tracks(p).states_posteriori = Tracks(p).states_priori + K*(Tracks(p).assigned_meas' - Tracks(p).y_cam_predicted);
       state_history(it, p, :) = Tracks(p).states_posteriori(:);
    end



elseif (camera_alone == 1)
    for p = 1:T
       K = Tracks(p).P_priori*((Tracks(p).H)')*inv((Tracks(p).H)*(Tracks(p).P_priori)*(Tracks(p).H)' +Tracks(p).R);
       Tracks(p).P_posteriori = [];
       Tracks(p).P_posteriori = (eye(nStates) - K*(Tracks(p).H))*Tracks(p).P_priori*(eye(nStates) - K*(Tracks(p).H))'+K*Tracks(p).R*K';
       Tracks(p).states_posteriori = [];
       Tracks(p).states_priori = Tracks(p).states_priori(:);
       Tracks(p).states_posteriori = Tracks(p).states_priori + K*(Tracks(p).assigned_meas' - Tracks(p).y_cam_predicted);
       state_history(it, p, :) = Tracks(p).states_posteriori(:);
    end


else
    for p = 1:T
       K = Tracks(p).P_priori*((Tracks(p).H)')*inv((Tracks(p).H)*(Tracks(p).P_priori)*(Tracks(p).H)' +Tracks(p).R);
       Tracks(p).P_posteriori = [];
       Tracks(p).P_posteriori = (eye(nStates) - K*(Tracks(p).H))*Tracks(p).P_priori*(eye(nStates) - K*(Tracks(p).H))'+K*Tracks(p).R*K';
       Tracks(p).states_posteriori = [];
       Tracks(p).states_priori = Tracks(p).states_priori(:);
       Tracks(p).states_posteriori = Tracks(p).states_priori + K*(Tracks(p).assigned_meas' - Tracks(p).y_rad_predicted);
       state_history(it, p, :) = Tracks(p).states_posteriori(:);
    end
end

rad = [];



end






T_final = size(state_history, 2);  % final number of tracks (maybe <= T)
num_steps = it;                     % number of time steps actually recorded

% Simple static plot of full trajectories
figure; hold on; grid on;
for p = 1:T_final
    xs = squeeze(state_history(1:num_steps, p, 1));
    ys = squeeze(state_history(1:num_steps, p, 2));
    % only plot if there is at least one nonzero row
    if any(xs) || any(ys)
        plot(xs, ys, '-o', 'LineWidth', 1.5);
    end
end
xlabel('X'); ylabel('Y'); title('Tracks over Time'); axis equal;

% Simple animation (no cells, numeric indexing)
figure('Color','w');
xlim([-100 100]); ylim([-100 100]);
xlabel('X'); ylabel('Y'); grid on; hold on;
for tstep = 1:num_steps
    clf; hold on; grid on;
    for p = 1:T_final
        xs = squeeze(state_history(1:tstep, p, 1));
        ys = squeeze(state_history(1:tstep, p, 2));
        if any(xs) || any(ys)
            plot(xs, ys, '-o', 'LineWidth', 1.5);
        end
    end
    title(sprintf('Tracker at time step %d / %d', tstep, num_steps));
    xlim([-100 100]); ylim([-100 100]); axis equal;
    drawnow;
    pause(0.08);
end
