function h1 = domaindisplay(h,k,R_tmp,yaw,Color)
%Input(x,y,R,yaw,color)
%   Detailed explanation goes here
x=[];y=[];
for i = 0:89
    x=[x;R_tmp(3)*sind(i)];
    y=[y;R_tmp(1)*cosd(i)];
end
for i = 90:179
    x=[x;R_tmp(3)*sind(i)];
    y=[y;R_tmp(2)*cosd(i)];
end
for i = 180:269
    x=[x;R_tmp(4)*sind(i)];
    y=[y;R_tmp(2)*cosd(i)];
end
for i = 270:359
    x=[x;R_tmp(4)*sind(i)];
    y=[y;R_tmp(1)*cosd(i)];
end
tmpx=x;
x=x*cosd(yaw)+y*sind(yaw);
y=y*cosd(yaw)-tmpx*sind(yaw);
h1=fill3(x+h,y+k,zeros(size(x)),Color,'edgealpha',1);
set(h1,'FaceAlpha',0.5);
end

