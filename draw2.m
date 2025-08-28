% close all
%% subplot(1): trajectory & obstacles
obstacle_color = [255 0 0]/255;
figure(2)
subplot('Position',[0.0686204431736955 0.348877374784111 0.429433722500199 0.583657167530225])
    xlabel('x (n mile)');
    ylabel('y (n mile)');
hold on
plot(start_point(1),start_point(2),'s','color',[0 0 255]/255,'MarkerSize',7,'MarkerFaceColor',[0 0 255]/255); % start point
plot(end_point(1),end_point(2),'p','color',[0 0 255]/255,'MarkerSize',12,'MarkerFaceColor',[0 0 255]/255);   % target point

% dummy plots for legend handles
i = -10; j = -10;
plot(i,j,'o','color',obstacle_color,'MarkerSize',7,'MarkerFaceColor',obstacle_color);       % static obstacle
plot(i,j,'-o','color',[0 0 0],'LineStyle','--','MarkerSize',7,'MarkerFaceColor',[1 1 1]);   % detection area
plot(i,j,'color',[0 0 255]/255,'MarkerSize',7,'MarkerFaceColor',[1 1 1],'LineWidth',1);     % path by own algorithm
shipDisplay3([0 0 0],i,j,0,1,[0 1 0]);                                                      % own ship

% draw first obstacle with both radius and detection area
for i = 1
    x = mat_point(i,1);
    y = mat_point(i,2);
    r = mat_point(i,3);
    d = mat_point(i,3)*mat_point(i,5);
    h2 = rectangle('Position',[x-r,y-r,2*r,2*r],'Curvature',[1,1],'facecolor',obstacle_color); % obstacle (circle)
    h3 = rectangle('Position',[x-d,y-d,2*d,2*d],'Curvature',[1,1],'LineStyle','--');           % detection area (dashed circle)
    % 'facecolor' = fill color, 'edgecolor' = outline color
end

% draw remaining obstacles
for i = 2:size(mat_point,1)
    x = mat_point(i,1);
    y = mat_point(i,2);
    r = mat_point(i,3);
    d = mat_point(i,3)*mat_point(i,5);
    rectangle('Position',[x-r,y-r,2*r,2*r],'Curvature',[1,1],'facecolor',obstacle_color); % obstacle
    rectangle('Position',[x-d,y-d,2*d,2*d],'Curvature',[1,1],'LineStyle','--');           % detection area
end

axis equal
grid on
axis([0 map_size(1) 0 map_size(2)]);
legend1 = legend('start','target','static obstacles','detection area','path by own algorithm','own ship','Location','SouthEast');
set(legend1,'Position',[0.322658966237086 0.232429764580638 0.17964731814842 0.252224199288256]);
set(gca,'FontSize',10)

%% subplot(2): rudder input vs time
subplot('Position',[0.553551686703887 0.739205526770294 0.334524660471765 0.195164075993097])
    xlabel('time (s)');
    ylabel('rudder (deg)');
axis([-inf inf -35 35]);
set(gca,'FontSize',10)

%% subplot(3): course angle vs time
subplot('Position',[0.554653817490069 0.349740932642487 0.333809864188702 0.27979274611399])
    xlabel('time (s)');
    ylabel('course angle (deg)');
axis([-inf inf 0 360]);
set(gca,'FontSize',10)