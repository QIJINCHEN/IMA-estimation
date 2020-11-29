function q = rv2quat(rv)
% -------------------------------------------------------------------------
%RV2QUAT The attitude quaternion in terms of rotation vector
%	q = rv2quat(rv)
%
%INPUTS:
%	rv = rotation vecotr, 3-by-1 vector
%OUTPUTS:
%   q = attitude quaternion
% 
% Reference:  E.-H. Shin, "Estimation techniques for low-cost inertial navigation," 
%             PhD Thesis, Deparment of Geomatics Engineering, 2005.
%             pp.12-13
% -------------------------------------------------------------------------
% Author:
% Qijin Chen, GNSS Research Center, Wuhan University, China.;
% chenqijin@whu.edu.cn;
% Nov. 2019;
% -------------------------------------------------------------------------
% q = zeros(4,1);
% mag2 = rv(1)*rv(1) + rv(2)*rv(2) + rv(3)*rv(3);
% if mag2 < 1.0e-8
%     q(1) = 1 - mag2 * (1/8 - mag2/384 + mag2*mag2/46080.0); 
%     s = 1/2 - mag2 * (1/48 - mag2/3840 + mag2*mag2/645120.0);
% else
%     mag = sqrt(mag2);
%     q(1) = cos(mag/2.0);
%     s = sin(mag/2.0)/mag;
% end
% q(2) = s*rv(1);
% q(3) = s*rv(2);
% q(4) = s*rv(3);

mag2 = rv(1)^2 + rv(2)^2 + rv(3)^2;
if mag2 < pi^2 %1.0e-8  
	mag2 = mag2/4.0;	
	c = 1.0 - mag2/2.0 * (1.0 - mag2/12.0 * (1.0 - mag2/30.0 ));
	s = 1.0 - mag2/6.0 * (1.0 - mag2/20.0 * (1.0 - mag2/42.0 ));	
	q = [c; s * 0.5 * rv(1); s * 0.5 * rv(2); s * 0.5 * rv(3) ];
else
	mag = sqrt(mag2);
	s_mag = sin(mag/2);	
	q = [ cos(mag/2);
        rv(1)*s_mag/mag;
        rv(2)*s_mag/mag;
        rv(3)*s_mag/mag ];
    
    if q(1) < 0
        q = -q;
    end
end