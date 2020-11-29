function [lat, lon] = qne2bl(q_ne)
% -------------------------------------------------------------------------
%QNE2BL Compute latitude and longitude from the quaternion q_ne
%	[lat, lon] = qne2bl(q_ne)
%
%INPUTS:
%	q_ne = the quaternion q_ne
%OUTPUTS:
%	1. lat = latitude in radians
%	2. lat = longitude in radians
% 
% Reference: E.-H. Shin, "Estimation techniques for low-cost inertial navigation," 
%            PhD Thesis, Deparment of Geomatics Engineering, 2005.
%            pp.23
% 
% see also qne
% -------------------------------------------------------------------------
% Author:
% Qijin Chen, GNSS Research Center, Wuhan University, China.;
% chenqijin@whu.edu.cn;
% Nov. 2019;
% -------------------------------------------------------------------------
lat = -2*atan(q_ne(3)/q_ne(1))-pi/2;
lon = 2*atan2(q_ne(4), q_ne(1));
end