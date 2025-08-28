function target_ship = init_ship(own_ship,situation,distance)
%INIT_SHIP 此处显示有关此函数的摘要
%   此处显示详细说明
global parameter
switch situation
    case 'head-on'
tmp_angle                = -2+4*rand;
target_ship.position     = [sind(own_ship.yaw+tmp_angle) cosd(own_ship.yaw+tmp_angle)]*distance+own_ship.position;
target_ship.yaw          = own_ship.yaw-2+4*rand+180;
target_ship.speed        = 5+(10-5)*rand; %16 knot
    case 'crossing1'
tmp_angle                = 90*rand;
target_ship.position     = [sind(own_ship.yaw+tmp_angle) cosd(own_ship.yaw+tmp_angle)]*distance+own_ship.position;
target_ship.yaw          = own_ship.yaw+tmp_angle+180+45;
target_ship.speed        = 5+(10-5)*rand; %16 knot
    case 'crossing2'
tmp_angle                = -90*rand;
target_ship.position     = [sind(own_ship.yaw+tmp_angle) cosd(own_ship.yaw+tmp_angle)]*distance+own_ship.position;
target_ship.yaw          = own_ship.yaw+tmp_angle+180-45;
target_ship.speed        = 5+(10-5)*rand; %16 knot
    case 'overtaking'
tmp_angle                = -2+4*rand;
target_ship.position     = [sind(own_ship.yaw+tmp_angle) cosd(own_ship.yaw+tmp_angle)]*distance+own_ship.position;
target_ship.yaw          = own_ship.yaw+tmp_angle;
target_ship.speed        = 5+(6-5)*rand; %16 knot
    case 'others'
tmp_angle                = 360*rand;
target_ship.position     = [sind(tmp_angle) cosd(tmp_angle)]*distance+own_ship.position;
target_ship.yaw          = tmp_angle;
target_ship.speed        = 5+(10-5)*rand; %16 knot
end
target_ship.v            = 0;
target_ship.data         = [...                        data = [ U K T n3]
    6     0.08    20    0.4   
    9     0.18    27    0.6
    12    0.23    21    0.3 ];
% interpolate to find K and T as a function of U
target_ship.k            = interp1(target_ship.data(:,1),target_ship.data(:,2),target_ship.speed,'linear','extrap');
target_ship.T            = interp1(target_ship.data(:,1),target_ship.data(:,3),target_ship.speed,'linear','extrap');
target_ship.n3           = interp1(target_ship.data(:,1),target_ship.data(:,4),target_ship.speed,'linear','extrap');
target_ship.Ddelta       = 10*pi/180;  % max rudder rate (rad/s)
target_ship.delta        = 30*pi/180;  % max rudder angle (rad)
target_ship.length       = 100;
target_ship.yaw_rate     = 0;
target_ship.rudder       = 0;
target_ship.rudder_rate  = 0;
target_ship.gamma        = 1;
target_ship.lamda        = log(1/parameter.minpotential4ship-1)/((parameter.shipdomain)^2-1);
target_ship.situation    = 0;
target_ship.predicted_position = [];
target_ship.shipdomain   = parameter.situs0*target_ship.length/parameter.scale;
end

