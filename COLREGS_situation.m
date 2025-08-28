function situation = COLREGS_situation(own_ship,target_ship)
h=target_ship.position(1);
k=target_ship.position(2);
h=h-own_ship.position(1);
k=k-own_ship.position(2);
angle = atan2(h,k)*180/pi;
angle = angle-own_ship.yaw;
angle = mod(angle,360);
situation = 0;
if (360-6 <= angle)||(angle <= 6)
    situation = 1;%head-on
end
if ((6 < angle)&&(angle <= 112.5))
    situation = 2;%crossing give-way
end
if ((360-6 > angle)&&(angle >= 360-112.5))
    situation = 3;%crossing stand-on
end
if ((360-112.5 > angle)&&(angle > 112.5))
    situation = 4;% overtaking
end
end