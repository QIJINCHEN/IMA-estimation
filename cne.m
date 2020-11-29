function C_ne = cne(lat, lon)
% -------------------------------------------------------------------------
%CNE The transformation matrix from local geodetic frame (n-frame) to
%        ECEF frame (e-frame) -NED system
%	C_ne = cne(lat, lon)
%
%INPUTS:
%	1. lat = latitude in radians
%	2. lat = longitude in radians
%OUTPUTS:
%   C_ne = transformation matrix from n-frame to e-frame.
% 
% Reference: R. M. Rogers, Applied mathematics in integrated navigation systems, 
%            3rd ed. Reston, VA: American Institute of Aeronautics and
%            Astronautics, 2007. pp57
% -------------------------------------------------------------------------
% Author:
% Qijin Chen, GNSS Research Center, Wuhan University, China.;
% chenqijin@whu.edu.cn;
% Nov. 2019;
% -------------------------------------------------------------------------
s_lat = sin(lat);
c_lat = cos(lat);
s_lon = sin(lon);
c_lon = cos(lon);

C_ne = [-s_lat*c_lon,  -s_lon,  -c_lat*c_lon;
     -s_lat*s_lon,  c_lon,  -c_lat*s_lon;
     c_lat,   0.0,  -s_lat];
end