function q_ne = qne(lat, lon)
% -------------------------------------------------------------------------
%QNE The quaternion corresponding to C_ne (transformation matrix from n-frame to e-frame) 
%	q_ne = qne(lat, lon)
%
%INPUTS:
%	1. lat = latitude in radians
%	2. lat = longitude in radians
%OUTPUTS:
%	q_ne = the quaternion q_ne
% 
% Reference: E.-H. Shin, "Estimation techniques for low-cost inertial navigation," 
%            PhD Thesis, Deparment of Geomatics Engineering, 2005.
%            pp.23
% 
% see also cne
% -------------------------------------------------------------------------
% Author:
% Qijin Chen, GNSS Research Center, Wuhan University, China.;
% chenqijin@whu.edu.cn;
% Nov. 2019;
% -------------------------------------------------------------------------
s1 = sin(lon/2);
c1 = cos(lon/2);
s2 = sin(-pi/4-lat/2);
c2 = cos(-pi/4-lat/2);

q_ne = [c1 * c2; 
    -s1 * s2;
    c1 * s2; 
    c2 * s1];
end