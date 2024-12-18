function [FPA, RNG, D_OBS, ALT, warn] = fcn(OBS, CURRENT, TARGET)

% Guidance Command
% #codegen

R = 6.371e6; % Earth's radius in meters
d2r = pi/180;

% Obstacle Location
LAT_OBS = OBS(1);
LON_OBS = OBS(2);

% Obstacle Threshold
thres = 2000; % 2Km

% Target Location
LAT_TARGET = TARGET(1);
LON_TARGET = TARGET(2);
ELEV_TARGET = TARGET(3);

% Current Location
ELEV_CUR = CURRENT(1);
LAT_CUR = CURRENT(2);
LON_CUR = CURRENT(3);

% distance to target
u1 = LAT_CUR*d2r;
l1 = LON_CUR*d2r;
u2 = LAT_TARGET*d2r;
l2 = LON_TARGET*d2r;
dh = abs(ELEV_TARGET-ELEV_CUR);
du = u2-u1;
dl = l2-l1;
a = sin(dl/2)^2 + cos(u1)*cos(u2)*sin(du/2)^2;
c = 2*atan2(sqrt(a),sqrt(1-a));
d = R*c; % Horizontal distance (in m)

% distance from the obstacle
u3 = LAT_OBS*d2r;
l3 = LON_OBS*d2r;
du = u3-u1;
dl = l3-l1;
a = sin(dl/2)^2 + cos(u1)*cos(u3)*sin(du/2)^2;
c = 2*atan2(sqrt(a),sqrt(1-a));
d_obs = R*c; % Horizontal distance (in m)

% current range (from target) - range > distance
range = sqrt(d^2+dh^2);

% calculate commanded flight path setpoint based on d_obs, range
if abs(d_obs)>=thres
    FPA_CMD = atan(dh/d);
    w = 0;
else
    % When in the obstacle zone, trigger warning
    w = 1;
    FPA_CMD = 0;
end

% output variables
FPA = FPA_CMD;
RNG = range;
D_OBS = d_obs;
warn = w;
ALT = ELEV_CUR;
end