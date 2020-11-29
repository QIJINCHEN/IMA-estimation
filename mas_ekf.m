function nav = mas_ekf(data_ains, cfg)
% -------------------------------------------------------------------------
%MASEKF implementation of the mounting angle estimation Kalman filtering 
%	dcm = eul2dcm(roll, pitch, heading)
%INPUTS:
%   1. imu_ains = the data used as input to DR and mounting angle
%              estimator. refer to table 2 for the format definition
%   2. cfg = the configuration
%OUTPUTS:
%	1. nav = Nx21 matrix, each column is defined as
%       nav(i,1) = time in seconds
%       nav(i,2) = distance in meters
%       nav(i,3:5) = latitude(in radians),longitude(in radians), height
%                   (in meters); 
%       nav(i,6:7) = pitch-mounting angle(in radians),heading-mounting angle
%                   (in radians);
%       nav(i,8:10) = estimated attitude errors in GNSS/INS smoothing roll, 
%                     pitch and heading angle (in radians); 
%       nav(i,11) = scale factor error of the distance measurement. 
% -------------------------------------------------------------------------
% Author:
% Qijin Chen, GNSS Research Center, Wuhan University, China.;
% chenqijin@whu.edu.cn;
% Nov. 2019;
% -------------------------------------------------------------------------

ds = [0; diff(data_ains(:,22))];  % distance increment between two epochs. 

% initialize the dead reckoning positioning. 
[nav, P, x, Q, G, Phi, datum] = initDR(data_ains, cfg);
cbv = eye(3);   % initialize the mounting angles as zeros

% progress bar
num = size(data_ains,1);
if num > 100
    fric = floor(num/100);
    handle = waitbar(0, 'DR aided by GNSS/INS position!', 'Name', 'IMU Mounting Angle Estimator');
    WaitTitle = 'Filtering...';
end

% DR/AINS position filtering
s_prev = data_ains(1,22);
for idx = 2:size(data_ains,1)
    
    % show current progress
    if num > 100
        if mod(idx, fric) == 0
            waitbar(idx/num, handle, WaitTitle);
        end
    end
    meas = data_ains(idx,:);
    cbn = eul2dcm(meas(10), meas(11), meas(12));

    % feedback the estimated mounting angles to compensate the GNSS/INS attitude
    cvn = cbn*cbv'; 
    
    % distance increment resolved in navigation frame 
    ds_n = cvn*[ds(idx); 0; 0]; 
    
    % the radii of curvature along lines of constant longitude and
    % latitude.
    Rm = datum.a*(1-datum.e2)/((1-datum.e2 * sin(data_ains(idx,2))^2)^(3/2));  
    Rn = datum.a/sqrt(1-datum.e2 * sin(data_ains(idx,2))^2);
    
    % DR position update
    nav(idx, 3) = nav(idx-1,3) + ds_n(1)/(Rm + data_ains(idx,4));
    nav(idx, 4) = nav(idx-1,4) + ds_n(2)/(Rn + data_ains(idx,4))/cos(data_ains(idx,2));
    nav(idx, 5) = nav(idx-1,5) - ds_n(3);

    % Set the state transition matrix PHI
    M = [0 0; 0 -abs(ds(idx)); abs(ds(idx)) 0];
    Phi(1:3,4:5) = -cvn*M;
    Phi(1:3,6:8) = cp_form(ds_n);
    Phi(1:3,9) = ds_n(:);

    % Kalman filter prediction, time update of the state and P matrix
    [x, P] = Kalman_predict(x, P, Phi, G, Q);
    
    % Kalman filter measurement update
    % The aided INS positioning solution is used as measurement update for
    % the DR system
    if abs(data_ains(idx,22)-s_prev) > cfg.kf.up_interval  
        r_gps_e = blh2xyz(datum.e2, Rn, data_ains(idx,2:4)');
        r_ins_e = blh2xyz(datum.e2, Rn, nav(idx,3:5)');
        la_r = zeros(3,1);
        C_en = cne(data_ains(idx,2), data_ains(idx,3))';
        z = C_en * (r_ins_e - r_gps_e) + la_r;
        
        % Measurement equations  
        Hm = zeros(3,9);
        Hm(1:3,1:3) = eye(3);      
        inno = z - Hm * x;    % innovation 
        R = diag((cfg.kf.sf_R_pos*data_ains(idx,13:15)).^2);
        [x, P, pdf] = Kalman_update(x, P, inno, Hm, R);
        if pdf ~= 0
            return
        end
             
        % position feedback
        d_lat = x(1)/(Rm + nav(idx,5));
        d_lon = x(2)/(Rn + nav(idx,5))/cos(nav(idx,3));
        d_theta = [d_lon * cos(nav(idx,3)); -d_lat; -d_lon * sin(nav(idx,3))];             
        qn = rv2quat(-d_theta);
        q_ne = qne(nav(idx,3), nav(idx,4));
        q_ne = quatprod(q_ne, qn);
        [nav(idx,3), nav(idx,4)] = qne2bl(q_ne);
        nav(idx,5) = nav(idx,5) + x(3);
        
        % Update the mounting angle estimates
        cvv = eul2dcm(0, x(4), x(5));
        cbv = cvv*cbv;
        
        % set the state components as zeros after feedback
        x(1:5,1) = zeros(1,5);
    end
    [~, nav(idx,6), nav(idx,7)] = dcm2eul(cbv);
    nav(idx,8:10) = x(6:8);
    nav(idx,11) = x(9);  
end

% close progress bar
if num > 100
    close(handle);
end
end

% Navigation initialization
function [nav, P, x, Q, G, Phi, datum] = initDR(imu, cfg)

datum = earthModel();

% initialize the DR position with adied INS positioning solutions
nav = zeros(length(imu), 11);	
nav(:,1:2) = [imu(:,1), imu(:,22)];    % time and distance index.
nav(1,3:5) = imu(1,2:4);    % initialize the DR position from GNSS/INS smoothing position

% Initialize the state error covariance matrix, P0
P = zeros(9,9);
P(1:3,1:3) = diag(1.0*(imu(1,13:15)).^2);     % position !!!! 1.0
P(4:5,4:5) = diag(cfg.ini_ma_std.^2);           % mounting angles
P(6:8,6:8) = diag((cfg.attstd_scale*imu(1,19:21)).^2);  % attitude uncertainty
P(9,9) = cfg.ini_odosf_std^2;	% incremental distance scale factor error

% state vector initialization
x = zeros(9,1);	

% set system noise covariance matrix, Q, time-invariant
Q = diag([cfg.q_ma_p;
    cfg.q_ma_y;
    cfg.q_arw;
    cfg.q_arw;
    cfg.q_arw;
    cfg.q_odosf].^2)*cfg.q_dt;

% discrete-time system noise distribution matrix
G = zeros(9,6);
G(4:9, 1:6) = eye(6);

% initilization of the discrete-time transition matrix
Phi = eye(9);
end

% Kalman Filter prediction
function [x_pred, P_pred] = Kalman_predict(x, P, PHI, Gamma, Qw)
x_pred = PHI * x;
P_pred = PHI * P * PHI' + Gamma * Qw * Gamma';
end

% Kalman filter measurement update
function [x_up, P_up, pdf] = Kalman_update(x, P, inno, H, R)
PHt = P * H'; 
HPHt = H * PHt;
RHPHt = R + HPHt;
[U, pdf] = chol(RHPHt);

if pdf == 0 % positive definite
    U = inv(U);
    U = U * U';
    K = PHt * U;
    dx = K * inno;
    x_up = x + dx;
    IKH = eye(length(x)) - K * H;
    P_up = IKH * P * IKH' + K * R * K';
else
    x_up = 0;
    P_up = zeros(5);
end

end
