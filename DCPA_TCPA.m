function [DCPA,TCPA] = DCPA_TCPA(ship,ship_t)
%DCPA Summary of this function goes here
%   Detailed explanation goes here
% TCPA tcpasition);
R_t       = norm(ship.position-ship_t.position);
v_xr      = ship_t.speed*sind(ship_t.yaw)+ship_t.v*cosd(ship_t.yaw)-ship.speed*sind(ship.yaw)-ship.v*cosd(ship.yaw);
v_yr      = ship_t.speed*cosd(ship_t.yaw)-ship_t.v*sind(ship_t.yaw)-ship.speed*cosd(ship.yaw)+ship.v*sind(ship.yaw);
V_r       = sqrt(v_xr^2+v_yr^2);
x_t       = ship_t.position(1);
y_t       = ship_t.position(2);
x         = ship.position(1);
y         = ship.position(2);
psi_r     = atan2(v_xr,v_yr);
psi_r     = psi_r*180/pi;
alp_t     = atan2((x_t-x),(y_t-y));
alp_t     = alp_t*180/pi;
flip1=0;
if psi_r-alp_t <90 && psi_r-alp_t>-90
    flip1=1;
end
% if psi_r<0
%     psi_r=psi_r+360;
% end
if v_xr>=0 && v_yr>=0
    psi_r = psi_r+0;
end
if (v_xr<0 && v_yr<0)||(v_xr>=0 && v_yr<0)
    psi_r = psi_r+180;
end
if v_xr<0 && v_yr>=0
    psi_r = psi_r+360;
end
% if psi_r>=360
%     psi_r = psi_r-360;
% end
% if alp_t<0
%     alp_t=alp_t+360;
% end
if (x_t-x)>=0 && (y_t-y)>=0
    alp_t = alp_t+0;
end
if (x_t-x)<0 && (y_t-y)>=0
    alp_t = alp_t+360;
end
if ((x_t-x)<0 && (y_t-y)<0)||((x_t-x)>=0 && (y_t-y)<0)
    alp_t = alp_t+180;
end
C_t       = ship_t.yaw-ship.yaw;
DCPA      = abs(R_t*sind(psi_r-alp_t-180));
TCPA      = abs(R_t*cosd(psi_r-alp_t-180)/V_r);
if flip1
    TCPA =-TCPA;
end
end

