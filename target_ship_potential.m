function z = target_ship_potential(own_ship,target_ship)
%TARGET_SHIP_POTENTIAL 此处显示有关此函数的摘要
%   此处显示详细说明
h=own_ship.position(1);
k=own_ship.position(2);
global parameter
z_tmp=zeros(1,1);
for i=1:1
    h=own_ship.position(1);
    k=own_ship.position(2);
    h=h-target_ship.position(1);
    k=k-target_ship.position(2);
    angle = atan2(h,k)*180/pi;
    angle = angle-target_ship.yaw;
    yaw   = target_ship.yaw;
    tmp_h = h;
    h=k*sind(-yaw)+h*cosd(-yaw);
    k=-tmp_h*sind(-yaw)+k*cosd(-yaw);
    switch target_ship.situation
        case 1 %head-on
            R_tmp = parameter.situs1*target_ship.length/parameter.scale;
        case 2 %crossing give-wall
            R_tmp = parameter.situs2*target_ship.length/parameter.scale;
        case 3 %crossing stand-on
            R_tmp = parameter.situs3*target_ship.length/parameter.scale;
        case 0 %other
            R_tmp = parameter.situs0*target_ship.length/parameter.scale;
        case 4
            R_tmp = parameter.situs0*target_ship.length/parameter.scale;
    end
    if 0<=mod(angle,360) && mod(angle,360)<=90
        tmp=(k/R_tmp(1))^2+(h/R_tmp(3))^2;
        if tmp<=1
            c = 0;
        else
            c= (k/R_tmp(2))^2+(h/R_tmp(4))^2-1;
        end
    end
    if 90<mod(angle,360) && mod(angle,360)<=180
        tmp=(k/R_tmp(2))^2+(h/R_tmp(3))^2;
        if tmp<=1
            c = 0;
        else
            c= (k/R_tmp(2))^2+(h/R_tmp(4))^2-1;
        end
    end
    if 180<mod(angle,360) && mod(angle,360)<=270
        tmp=(k/R_tmp(2))^2+(h/R_tmp(4))^2;
        if tmp<=1
            c = 0;
        else
            c= (k/R_tmp(2))^2+(h/R_tmp(4))^2-1;
        end
    end
    if 270<mod(angle,360) && mod(angle,360)<=360
        tmp=(k/R_tmp(1))^2+(h/R_tmp(4))^2; 
        if tmp<=1
            c = 0;
        else
            c= (k/R_tmp(2))^2+(h/R_tmp(4))^2-1;
        end
    end
    z_tmp(i) = 1/(1+target_ship.gamma*exp(target_ship.lamda*c));
end
    z=max(z_tmp)*2;
end

