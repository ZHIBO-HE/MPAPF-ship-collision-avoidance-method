function [potential] = Copy_of_test4potential(axis)
%TEST4POTENTIAL Summary of this function goes here
%   Detailed explanation goes here
global mat_point
global end_point
global ship
global parameter
global targetship
tmp_ship=ship; 
X_asumd_leader = axis;
tmp_ship.position =axis;
sum_cost_obstacles=0;
potential = 0;
for j=1:size(mat_point,1)
    distance_LA2OB=norm(mat_point(j,1:2)-X_asumd_leader);
    tmp=mat_point(j,1:2)-X_asumd_leader;
        if distance_LA2OB>=mat_point(j,5)*mat_point(j,3)
            distance_LA2OB=inf;
        end
        if distance_LA2OB<=mat_point(j,5)*mat_point(j,3)
                    sum_cost_obstacles=sum_cost_obstacles+exp(-mat_point(j,6)*distance_LA2OB^2);
        end
end
control_action = (end_point-X_asumd_leader);
potential = -exp(-norm(-(control_action)).^(2)*parameter.endbeta)+sum_cost_obstacles;
for j=1:size(mat_point,1)
    distance_LA2OB=norm(mat_point(j,1:2)-tmp_ship.position);
        if distance_LA2OB>=mat_point(j,5)*mat_point(j,3)
            continue
        end
        if distance_LA2OB<=mat_point(j,5)*mat_point(j,3)
            obstacle.position = mat_point(j,1:2);
            obstacle.speed    = mat_point(j,7);
            obstacle.v        = 0;
            obstacle.yaw      = mat_point(j,8);
            [DCPA,TCPA]       = DCPA_TCPA(tmp_ship,obstacle);
            if TCPA >= 0
            potential         = potential+ exp(-mat_point(j,6)*DCPA^2)*1;
            end
        end
end
for j=1:size(targetship,2)
    distance_LA2OB=norm(targetship(j).position-tmp_ship.position);
        if distance_LA2OB>=parameter.safecv
            distance_LA2OB=inf;
        end
        if distance_LA2OB<=parameter.safecv && distance_LA2OB>parameter.dangercv
            target_ship=targetship(j);
            target_ship.postion=targetship(j).position;
            potential=potential+target_ship_potential(tmp_ship,target_ship);
        end
        if distance_LA2OB<=parameter.dangercv
            target_ship=targetship(j);
            target_ship.postion=targetship(j).position;
            target_ship.situation=0;
            potential=potential+target_ship_potential(tmp_ship,target_ship);
        end
end
for j=2:size(targetship,2)
    distance_LA2OB=norm(targetship(j).position-tmp_ship.position);
        if distance_LA2OB<=parameter.safecv*1.5
%             [DCPA,TCPA]       = DCPA_TCPA(tmp_ship,targetship(j));
            if (targetship(j).situation == 1 ||targetship(j).situation == 2) && (isright(ship,targetship(j)) == 1)
             c = (ship.position(2)-targetship(j).position(2))*tmp_ship.position(1)+...
                     (-ship.position(1)+targetship(j).position(1))*tmp_ship.position(2)+...
                     ship.position(1)*targetship(j).position(2)-...
                     ship.position(2)*targetship(j).position(1);
%             curve = tmp_ship.position(1)-tmp_ship.position(2);
            potential         = potential+ 1/(1+exp(-1*c));
            end
        end
end
end
