function em = earthModel()
% -------------------------------------------------------------------------
%EARTHMODEL WGS84 ellipsoid constants
% Reference: R. M. Rogers, Applied mathematics in integrated navigation systems, 
%           3rd ed. Reston, VA: American Institute of Aeronautics and Astronautics,
%           2007. pp.77
% -------------------------------------------------------------------------
% Author:
% Qijin Chen, GNSS Research Center, Wuhan University, China.;
% chenqijin@whu.edu.cn;
% Nov. 2019;
% -------------------------------------------------------------------------
em.a = 6378137.0;
em.b = 6356752.3142;
em.f = 1.0/298.257223563;
em.we = 7.292115147e-5; % earth rotation angular rate
em.e2 = 1 - (em.b/em.a)^2;
% em.gm = 3986004.418e+8;
% em.j2 = 1.082626683e-3;
% em.j3 = -2.5327e-6;
em.e_2 = 1 - (em.b/em.a)^2;
em.e_2_ = -1 + (em.a/em.b)^2;