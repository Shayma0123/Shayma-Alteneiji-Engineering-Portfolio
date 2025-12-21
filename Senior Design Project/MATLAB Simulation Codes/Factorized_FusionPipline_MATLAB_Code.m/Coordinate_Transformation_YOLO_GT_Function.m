function [d_x_y,Rcam, gt_data] = Coordinate_Transformation_YOLO_GT(cam_meas,gt_data, Z, K_intrinsic,YOLO_W,YOLO_L, R_new, T_new, pix_error )

Ext = [R_new T_new]; % Extrnsic calibration matric [R|t]

P_projective = K_intrinsic*Ext; % Projective transformation matrix 

% Cmaera image  resolution 
Orig_W = 4056;Orig_L = 3040;

% Undoing the image resizing done during YOLO procassing 
cam_meas(:,1) = cam_meas(:,1)*(Orig_W/YOLO_W);
cam_meas(:,2) = cam_meas(:,2)*(Orig_L/YOLO_L);



% H 3x3 homography matrix 
H = [P_projective(:,1) P_projective(:,2) P_projective(:,3)*Z+P_projective(:,4) ];


% 3 x n ( number of rows in the YOLO dataset) matrix
pixels = [cam_meas(:,1)'; 
          cam_meas(:,2)'; 
          ones(1, size(cam_meas,1))];

% solving for the [kx ; ky;k] vector
x_y_w = H\pixels;
% normalizing the vector by dividing by the 3rd element k
x_y = (x_y_w(1:2,:) ./ x_y_w(3,:))';

% Estimating the camera noise covariance matrix using the extrnsic
% calibration reprojection error

d_uv = [pix_error; pix_error; 1];

dx_y_1 = inv(H)*d_uv;

d_x_y = dx_y_1(1:2,:)/dx_y_1(3,:);

Rcam = [d_x_y(1)^2 0 ; 0 d_x_y(2)^2];

% Coordinate transforming the ground truth using the rotation atrix R, and
% the translational vector T 

% extracting the GT - x and y coordinate measurments in the camera coordinate
% frame 
u_v_gt = gt_data(:, 5:7)';

% Obtaining the R camera -> radar matrix 

Trc = -R_new*T_new ;

% Obtaining the T camera -> radar vector 
Rrc = R_new';

% obtaining the GT measurments in the radar coordinate frame 
x_y_gt_nor = (Rrc*u_v_gt + Trc)';

gt_data(:, 5:7) = x_y_gt_nor;

end
