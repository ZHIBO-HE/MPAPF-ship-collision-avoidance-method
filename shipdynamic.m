function ship = shipdynamic(ship,rudder,time)
%SHIPDYNAMIC 此处显示有关此函数的摘要
%   此处显示详细说明
global parameter
if nargin <=2
    time=parameter.time;
end
    phi=ship.speed*time/parameter.scale;
    if rudder ~= 0
    [~,u_out] = ode45(@ship_responding,[0 time],[ship.yaw_rate rudder]);
    else 
        u_out = 0;
    end
    ship.yaw  = ship.yaw+sum(u_out(:,1))*180/pi/size(u_out,1)*time;
    gamma     = ship.yaw;
    ship.yaw_rate    = u_out(end,1);
    ship.rudder      = rudder;
%     ship.rudder_rate = result_mat(3);
%     ship.position=ship.position+[phi*sind(gamma) phi*cosd(gamma)]+parameter.water*parameter.time;
ship.position=ship.position+[phi*sind(gamma) phi*cosd(gamma)];
%     ship.position=ship.position
end

