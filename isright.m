function out = isright(Ownship,Targetship)
%ISRIGHT 此处显示有关此函数的摘要
%   此处显示详细说明
out = 0;
h=Targetship.position(1);
k=Targetship.position(2);
h=h-Ownship.position(1);
k=k-Ownship.position(2);
angle = atan2(h,k)*180/pi;
angle = angle-Ownship.yaw;
angle = mod(angle,360);
if angle <=90 && angle>0
    out = 1;
end
if angle <=180 && angle>90
    out = 2;
end
if angle <=270 && angle>180
    out = 3;
end
if angle <360 && angle>=270
    out = 4;
end
end

