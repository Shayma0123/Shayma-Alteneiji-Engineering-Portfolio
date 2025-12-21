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
    
