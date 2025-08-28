function [collision, cvship] = iscollisionavoidacne(ship,targetship)
%ISCOLLISIONAVOIDACNE 此处显示有关此函数的摘要
%   此处显示详细说明
global parameter
cvship=0;
collision=0;
for i = 1:size(targetship,2)
    distance(1) = parameter.safecv;
    [~,tcpa] = DCPA_TCPA(ship,targetship(i));
    if tcpa >=0
    distance(2) = norm(ship.position-targetship(i).position);
    if distance(1) >=distance(2)
        distance(1) = distance(2);
        cvship = i;
        collision=1;
    end
    end
end

