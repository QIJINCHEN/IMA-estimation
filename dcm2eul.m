function [roll, pitch, heading] = dcm2eul(dc)
% -------------------------------------------------------------------------
%DCM2EUL Convert from direction cosine matrix to Euler angles (3-2-1) -NED
%        system
%	[roll, pitch, heading] = dcm2eul(dc)
%
%INPUTS:
%   dcm = the direction cosine matrix, 3-by-3
%OUTPUTS:
%	roll = roll angle in radians
%	pitch = pitch angle in radians
%	heading = heading angle radians 
% 
% Reference: E.-H. Shin, "Estimation techniques for low-cost inertial navigation," 
%            PhD Thesis, Deparment of Geomatics Engineering, 2005.
%            pp.17-18
% -------------------------------------------------------------------------
% Author:
% Qijin Chen, GNSS Research Center, Wuhan University, China.;
% chenqijin@whu.edu.cn;
% Nov. 2019;
% -------------------------------------------------------------------------

pitch = atan(-dc(3,1)/sqrt(dc(3,2)^2 + dc(3,3)^2));

if dc(3,1) <= -0.999
    roll = NaN;
    heading = atan2((dc(2,3)-dc(1,2)),(dc(1,3)+dc(2,2)));
    
elseif dc(3,1) >= 0.999
    roll = NaN;
    heading = pi + atan2((dc(2,3)+dc(1,2)),(dc(1,3)-dc(2,2)));
    
else
    roll = atan2(dc(3,2), dc(3,3));
    heading = atan2(dc(2,1), dc(1,1));
end
end