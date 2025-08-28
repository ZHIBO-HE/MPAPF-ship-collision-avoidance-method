function [potential] = PSO_MPC_Cost_Function(u)
%TEST4POTENTIAL Summary of this function goes here
%   Detailed explanation goes here
global mat_point
global end_point
global ship
global parameter
global targetship
tmp_ship=ship;
potential = 0;
for i = 1 : parameter.prediction_step
tmp_ship = shipdynamic(tmp_ship,u(i*2));
sum_cost_obstacles=0;
if i ==1
for j=1:size(mat_point,1) %potential based on DCPA for static obstacles
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
for j=2:size(targetship,2) %potential lead ship turn right
    distance_LA2OB=norm(targetship(j).position-tmp_ship.position);
        if distance_LA2OB<=parameter.safecv
%             [DCPA,TCPA]       = DCPA_TCPA(tmp_ship,targetship(j));
            if (targetship(j).situation == 1) && (isright(ship,targetship(j)) == 1)
             c = (ship.position(2)-targetship(j).position(2))*tmp_ship.position(1)+...
                     (-ship.position(1)+targetship(j).position(1))*tmp_ship.position(2)+...
                     ship.position(1)*targetship(j).position(2)-...
                     ship.position(2)*targetship(j).position(1);
            potential         = potential+ 1/(1+exp(-1*c));
            end
        end
end
end
for j=1:size(mat_point,1)
    distance_LA2OB=norm(mat_point(j,1:2)-tmp_ship.position);
        if distance_LA2OB>=mat_point(j,5)*mat_point(j,3)
            distance_LA2OB=inf;
        end
        if distance_LA2OB<=mat_point(j,5)*mat_point(j,3)
            sum_cost_obstacles=sum_cost_obstacles+exp(-mat_point(j,6)*distance_LA2OB^2)/parameter.prediction_step;
        end
control_action = (end_point-tmp_ship.position);
potential = potential+(-exp(-norm(-(control_action)).^(2)*parameter.endbeta)+sum_cost_obstacles)*1; 
end
for j=1:size(targetship,2)
    distance_LA2OB=norm(targetship(j).predicted_position(i,1:2)-tmp_ship.position);
        if distance_LA2OB>=parameter.safecv
            distance_LA2OB=inf;
        end
        if distance_LA2OB<=parameter.safecv && distance_LA2OB>parameter.dangercv
            target_ship=targetship(j);
            target_ship.postion=targetship(j).predicted_position(i,1:2);
            potential=potential+target_ship_potential(tmp_ship,target_ship);
        end
        if distance_LA2OB<=parameter.dangercv
            target_ship=targetship(j);
            target_ship.postion=targetship(j).predicted_position(i,1:2);
            target_ship.situation=0;
            potential=potential+target_ship_potential(tmp_ship,target_ship)/parameter.prediction_step;
        end
end
u(i*2)*180/pi;
end
potential;
end
