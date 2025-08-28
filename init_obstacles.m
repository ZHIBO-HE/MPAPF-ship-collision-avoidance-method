function mat = init_obstacles(static_obs_num,static_obs_area)
%INIT_OBSTACLES 此处显示有关此函数的摘要
%   此处显示详细说明
global parameter
global end_point
global start_point
mat                = zeros(sum(static_obs_num),8);
row                = 1;
for area = 1 : size(static_obs_num,1)
    for i = 1 : static_obs_num(area)
        while (1)
            tmp_x = static_obs_area(area,1)+(-static_obs_area(area,1)+static_obs_area(area,3))*rand;
            tmp_y = static_obs_area(area,2)+(-static_obs_area(area,2)+static_obs_area(area,4))*rand;
            tmp_r = parameter.minobstacle+(-parameter.minobstacle+parameter.maxobstacle)*rand;
            tmp   = 0;
            if norm([tmp_x,tmp_y]-end_point) <= tmp_r*parameter.safeobstacle || norm([tmp_x,tmp_y]-start_point) <=tmp_r*parameter.safeobstacle
                tmp = 1;
                continue
            end
            for j = 1 : size(mat,1)
                tmp_d = norm([tmp_x,tmp_y]-mat(j,1:2));
                if tmp_d <= (tmp_r+mat(j,3))*1.5
                    tmp = 1;
                    break
                end
            end
            if tmp == 0
            beta = -log(parameter.minpotential)/(tmp_r*parameter.safeobstacle)^2;
            mat(row,:) = [tmp_x,...
                          tmp_y,...
                          tmp_r,...
                          1.5,...
                          parameter.safeobstacle,...
                          beta,...
                          0,...
                          0];%X,Y,Radius,Safety margin,Detection zone
            row        = row+1;
            break
            end
        end
    end
end
end

