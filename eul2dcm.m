function dcm = eul2dcm(roll, pitch, heading)
% -------------------------------------------------------------------------
%EUL2DCM Convert from Euler angle to direction cosine matrix - NED system
%	dcm = eul2dcm(roll, pitch, heading)
%
%INPUTS:
%	1. roll = roll angle in radians
%	2. pitch = pitch angle in radians
%	3. heading = heading angle in radians
%OUTPUTS:
%   dcm = the direction cosine matrix, 3-by-3
% 
% Reference: P. G. Savage, Strapdown analytics - Part 1
%           Maple Plain, Minnesota: Strapdown Associates, 2007. pp3-33
% -------------------------------------------------------------------------
% Author:
% Qijin Chen, GNSS Research Center, Wuhan University, China.;
% chenqijin@whu.edu.cn;
% Nov. 2019;
% -------------------------------------------------------------------------
dcm = zeros(3);

cr = cos(roll); 
cp = cos(pitch); 
ch = cos(heading);
sr = sin(roll); 
sp = sin(pitch); 
sh = sin(heading);

dcm(1,1) = cp * ch ;
dcm(1,2) = -cr*sh + sr*sp*ch;
dcm(1,3) = sr*sh + cr*sp*ch ;

dcm(2,1) = cp * sh;
dcm(2,2) = cr*ch + sr*sp*sh;
dcm(2,3) = -sr * ch + cr * sp * sh;

dcm(3,1) = - sp;
dcm(3,2) = sr * cp;
dcm(3,3) = cr * cp;
end