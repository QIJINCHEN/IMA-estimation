function data_ains = dataPreproc(cfg)
% -------------------------------------------------------------------------
%DATAPREPROC Preprocess the GNSS/INS smoothing solution: 1) calculate the 
%            traveled distance from GNSS/INS smoothing position; 2) resample 
%            GNSS/INS smoothing solution along distance. Prepare data for
%            DR and mounting angle KF estimator.     
%	data_ains = dataPreproc(cfg)
%
%INPUTS:
%	cfg = the configuration
%OUTPUTS:
%   imu_ains = the data used as input to DR and mounting angle
%              estimator. refer to table 2 for the format definition
% -------------------------------------------------------------------------
% Author:
% Qijin Chen, GNSS Research Center, Wuhan University, China.;
% chenqijin@whu.edu.cn;
% Nov. 2019;
% -------------------------------------------------------------------------


% read the GNSS/INS smoothing solution; refer to table 1 for the format 
fid = fopen(cfg.fins, 'rb');
if fid == -1
    disp('Cannot open the file!');
    data_ains = [];
    return;
end
data = fread(fid, [21,inf], 'double')';
fclose(fid); clear fid;

% the segment of trajectory used to estimate the mounting angles
n1 = find(data(:,1)>cfg.session(1), 1);
n2 = find(data(:,1)>cfg.session(2), 1);
data = data(n1:n2, :);

% calculate velocity in v-frame to help computing the distance increment
v_v = zeros(size(data,1), 3);
for idx = 1:size(v_v,1)
    cbn = eul2dcm(data(idx,10)*pi/180.0, data(idx,11)*pi/180.0, data(idx,12)*pi/180.0);
    v_v(idx, :) = (cbn' * data(idx,7:9)')';
end
dt = diff(data(:,1));
ds = v_v(1:end-1,1).*dt;
vdist = [0; cumsum(ds(:))];
ts_vel = [data(:,1), vdist];

[~, m, ~] = unique(ts_vel(:,2));
ts_vel = ts_vel(m, :);
data = data(m,:);

% compute traveled distance from GNSS/INS smoothing position 
s1 = min(ts_vel(:,2)):cfg.fds:max(ts_vel(:,2));
t1 = interp1(ts_vel(:,2), ts_vel(:,1), s1);
t1 = t1(:);

pos = interp1(data(:,1), [data(:,5:6) data(:,4)], t1);
dpos = diff(pos);
ds = zeros(size(pos,1), 1);
for idx = 1:size(dpos,1)
    ds(idx+1,1) = sqrt(dpos(idx,1)*dpos(idx,1)+dpos(idx,2)*dpos(idx,2)+dpos(idx,3)*dpos(idx,3));
end
ts_pos = [t1, cumsum(ds)];

% resample the GNSS/INS smoothing solution according to the distance
data(:,12) = yawSmooth(data(:,12));
data = interp1(data(:,1),data, ts_pos(:,1));
data(:,12) = data(:,12)-floor(data(:,12)/360)*360;

n = isnan(data(:,1));
data(n, :) = [];

% Unit conversion
data(:,2:3) = data(:,2:3)*pi/180;
data(:,10:12) = data(:,10:12)*pi/180;
data(:,19:21) = data(:,19:21)*pi/180;

data_ains = [data, ts_pos(:,2)]; 
end

function Az = yawSmooth(Az)

N = length(Az);
A_threshold = 180;
for i = 2:N
    if Az(i) - Az(i-1) > A_threshold
        Az(i:N) = Az(i:N) - 360;
    elseif Az(i) - Az(i-1) < -A_threshold
        Az(i:N) = Az(i:N) + 360;
    end
end
end