function xyz = blh2xyz(e2, Rn, blh)
% -------------------------------------------------------------------------
%BLH2XYZ Convert from latitude, longitude and height
%        to Earth-centered, Earth-fixed (ECEF) cartesian coordinates.
%	xyz = blh2xyz(e2, Rn, blh)
%
%INPUTS:
%   1. e2 = square of the ellipticity of the reference Earth ellipse. 
%   2. blh  = position vector
%       blh(1) = latitude in radians
%       blh(2) = longitude in radians
%       blh(3) = height above ellipsoid in meters
%   3. Rn = the radii of curvature along lines of constant latitude, in
%   meters
%OUTPUTS:
%	xyz(1) = ECEF x-coordinate in meters
%	xyz(2) = ECEF y-coordinate in meters
%	xyz(3) = ECEF z-coordinate in meters
% -------------------------------------------------------------------------
% Author:
% Qijin Chen, GNSS Research Center, Wuhan University, China.;
% chenqijin@whu.edu.cn;
% Nov. 2019;
% -------------------------------------------------------------------------
c_lat = cos(blh(1));
s_lat = sin(blh(1));
c_lon = cos(blh(2));
s_lon = sin(blh(2));

Rn_h = Rn + blh(3);

xyz = [Rn_h*c_lat*c_lon;
    Rn_h * c_lat * s_lon;
    (Rn*(1-e2)+blh(3))*s_lat];
end