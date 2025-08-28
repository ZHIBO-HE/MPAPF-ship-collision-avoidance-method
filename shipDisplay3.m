 function [h1] = shipDisplay3(theta,xShip,yShip,zShip,scale,color)
% shipDisplay3([270 0 0],1,1,0,0.1,rand(1,3))
% theta: ship heading; theta(1): based on z; theta(2): based on y;
% theta(3): based on x;
% xShip: ship x position; yShip: ship y position; zShip: ship z postion;
% scale: scale of ship
% color: color of ship
% example：xy       = shipdisplay(theta);...
% otherships(j)     = fill(xy(1,:)+tmp_TTs(j,1),xy(2,:)+tmp_TTs(j,2),'b');
if isempty(zShip)
    zShip=0;
end
if isempty(scale)
    scale=1;
end
theta(1) =90-theta(1);
    %%top
    x           = [-0.5:0.01:0.5]*scale;
    y           = 0.125*sin(4*pi*x/scale)*scale.*(x>=-0.5*scale&x<-0.375*scale)...
                  +0.125*scale.*(x>=-0.375*scale&x<0.125*scale)...
                  +0.125*scale*cos(4*pi*x/3/scale-pi/6).*(x>=0.125*scale&x<=0.5*scale);
    x           = x';
    y2          = -y;
    y2          = y2';
    y           = y';
    tmpxy       = [x y];
    tmpxy1      = [x y2];
    tmpxy1      = flipud(tmpxy1);
    matrot_z    = [cosd(-theta(1)),-sind(-theta(1)),0;...
                   sind(-theta(1)),cosd(-theta(1)),0;
                   0,0,1];
    matrot_y    = [cosd(-theta(2)),0,sind(-theta(2));...
                   0,1,0;
                   -sind(-theta(2)),0,cosd(-theta(2))];
    matrot_x    = [1,0,0;...
                   0,cosd(-theta(3)),-sind(-theta(3));
                   0,sind(-theta(3)),cosd(-theta(3))];
    xy          = [tmpxy;tmpxy1];
        z           = ones(length(xy),1)*scale*0.1;
    xy          = [xy,z];
    xy          = xy*matrot_y*matrot_x*matrot_z;
%     tmpxy       = tmpxy*matrot_z;
%     tmpxy       = tmpxy';
%     tmpxy1      = tmpxy1*matrot_z;
%     tmpxy1      = tmpxy1';
%     tmpxy1      = fliplr(tmpxy1);
%     xy          = [tmpxy tmpxy1];                                           % origin point at (0,0)
    
    %%button
%     scale       = 0.8*scale;
%     x           = [-0.5:0.01:0.5]*scale;
%     y           = 0.125*sin(4*pi*x/scale)*scale.*(x>=-0.5*scale&x<-0.375*scale)...
%                   +0.125*scale.*(x>=-0.375*scale&x<0.125*scale)...
%                   +0.125*scale*cos(4*pi*x/3/scale-pi/6).*(x>=0.125*scale&x<=0.5*scale);
%     x           = x';
%     y2          = -y;
%     y2          = y2';
%     y           = y';
%     tmpxy       = [x y];
%     tmpxy1      = [x y2];
%     matrot      = [cosd(-theta),-sind(-theta);...
%                    sind(-theta),cosd(-theta);];
%     tmpxy       = tmpxy*matrot;
%     tmpxy       = tmpxy';
%     tmpxy1      = tmpxy1*matrot;
%     tmpxy1      = tmpxy1';
%     xy1          = [tmpxy tmpxy1];      
%     z1          = zeros(length(xy1));
    h1=fill3(xy(:,1)+xShip,xy(:,2)+yShip,xy(:,3)+zShip,color);% display the ship picture
    set(h1,'FaceAlpha',1);%画一个白船，上面再叠加一个
%     hold on
%     fill3(xy1(1,:)+xShip,xy1(2,:)+yShip,z1+zShip,'b');                                  % display the ship picture
%     xline(1:2:2*length(xy))=xy(1,:);xline(2:2:2*length(xy))=xy1(1,:);
%     yline(1:2:2*length(xy))=xy(2,:);yline(2:2:2*length(xy))=xy1(2,:);
%     zline(1:2:2*length(xy))=scale*0.1+zShip;zline(2:2:2*length(xy))=0+zShip;
%     line(xline,yline,zline,'LineWidth',4,'MarkerEdgeColor','b');
end