function [measurment, Rmat] = measument_struct(data_sorted,indices,k, range_res, rvel_res,azimuth_res, flag)

    if flag == 3

        indeces0 = data_sorted(indices, 3) == 1;% 1 encodes for radar meas.
        radar_indices = indices(indeces0);
        camera_indeces = setdiff(indices, radar_indices);
            
        measurment(k).y_rad_meas = data_sorted(radar_indices,6:8);

        % produces a 3D tensor, 3 x3 by the number of measurments 

        Rmat = Measurement_Covariance(measurment, range_res, rvel_res,azimuth_res, k);

        measurment(k).y_cam_meas = data_sorted(camera_indeces,6:7);

    elseif flag == 1

        measurment(k).y_rad_meas = [];
        measurment(k).y_cam_meas = data_sorted(indices,6:7);
        Rmat =[];

    elseif flag == 2 

        measurment(k).y_rad_meas = data_sorted(indices,6:8);
        Rmat = Measurement_Covariance(measurment, range_res, rvel_res,azimuth_res, k);
        measurment(k).y_cam_meas = [];
    end
end
    

function Rmat = Measurement_Covariance(measurment, range_res, rvel_res,azimuth_res, k)

    y_rad_mea_vect = measurment(k).y_rad_meas;

    [rows, ~] = size(y_rad_mea_vect);


    % tensor 3 x 3 x by the number of radar measurments 
    Rmat = zeros(3,3,rows);

   for q = 1:rows

       x = y_rad_mea_vect(q,1);
       y = y_rad_mea_vect(q,2);

       theta = atan2(y, x); 
       range = sqrt(x^2 + y^2);

       var_x = (cos(theta))^2*(range_res^2/4) - (range*sin(theta))^2*(azimuth_res^2/4);
       var_y = (sin(theta))^2*(range_res^2/4) + (range*cos(theta))^2*(azimuth_res^2/4);
       doppler_res = (rvel_res^2)/4;

       Rmat(:,:,q) = [var_x 0 0; 0 var_y 0; 0 0 doppler_res];              
   end
end
