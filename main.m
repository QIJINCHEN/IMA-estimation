function main()
% -------------------------------------------------------------------------
% This open-source program implementing the IMU mounting angle estimation
% KF may help the readers in understanding the algorithm designed in the 
% manuscript - 
%  Qijin Chen, Xiaoji Niu, Jingnan Liu, "Estimation of 
%  IMU Mounting Angles for Land Vehicular GNSS/INS Integrated System"
% 
%  Please read the user manual for the definitions of the raw data format. 
% -------------------------------------------------------------------------
% Author:
% Qijin Chen, GNSS Research Center, Wuhan University, China.;
% chenqijin@whu.edu.cn;
% Nov. 2019;
% -------------------------------------------------------------------------
% close all; clear; clc
% step 1: set configuration 
cfg.fins = '.\data\ains_imu2.bin';   % raw GNSS/INS smoothing result; change to process different IMU data.
cfg.imutype = 'imu2';       % IMU type; 'imu1', 'imu2', 'imu3' for the three simulated IMUs
cfg.session = [800; 1200];  % time segment of the trajectory used as input to the KF estimator

% preprocess the GNSS/INS smoothing solution :1) derive traveled distance
% from GNSS/INS position; 2) resample GNSS/INS smoothing solution in
% distance domain. 
cfg.fds = 0.1; % distance interval for resample the raw GNSS/INS smoothing solution
data_ains = dataPreproc(cfg);

% Kalman filter tuning, P0, Q, R matrice.
cfg = paraTuning(cfg);

% step 2: KF estimator
nav = mas_ekf(data_ains, cfg);

% step 3: show results
cfg.plot_ma = 1;
cfg.plot_att_err = 1;
cfg.plot_odosf = 1;
plotResult(nav, cfg);


end

function cfg = paraTuning(cfg)
%% P0 matrix
% Uncertainty of the initial mounting angles(zeros). 
cfg.ini_ma_std = [1; 2] *pi/180.0;       % 1 deg for pitch-, 2 deg for heading-mounting angle, respectively.

% Uncertainty of the scale factor error of the distance measurement derived
% from GNSS/INS smoothing position. 
cfg.ini_odosf_std = 100 *1.0e-6;         % 100 ppm

% Scale to enlarge the initial attitude uncertainty. 
cfg.attstd_scale = 1.0;

%% Q matrix
% ARW: angular random walk, i.e., white noise. of the gyro outputs
switch cfg.imutype
    case 'imu1'
        cfg.q_arw = 0.0022 *pi/180.0/60; % deg/sqrt(h) -> rad/sqrt(s)       
    case 'imu2'
        cfg.q_arw = 0.15 *pi/180.0/60;
    case 'imu3'
        cfg.q_arw = 0.1 *pi/180.0/60;
    otherwise
        cfg.q_arw = 0.1 *pi/180.0/60;
end
% covariance of the noise in modeling the pitch- and heading-mounting angles
cfg.q_ma_p = 0.001 *pi/180.0/60;    % for pitch-mounting angle
cfg.q_ma_y = 0.001 *pi/180.0/60;    % for heading-mounting angle
cfg.q_odosf = 1.0 *1.0e-6;          % for scale factor error of the distance measurement
cfg.q_dt = 0.1;                     % average time interval between two DR epoch. 

%% R matrix
cfg.kf.up_interval = 0.2;   % the distance interval to use GNSS/INS position update.
cfg.kf.sf_R_pos = 3;      % scale to enlarge the uncertainty of GNSS/INS position, R
end