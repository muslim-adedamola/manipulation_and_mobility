clear;
L_1 = 0.5;          %link length L1
L_2 = 0.4;          %link length L2
R_obstacle = 0.2;   %Radius of obstacles
x_obst1 = -0.6;     %x-coordinate of obstacle 1
y_obst1 = 0.7;      %y-coordinate of obstacle 1
x_obst2 = 0.6;      %x-coordinate of obstacle 2
y_obst2 = 0.7;      %y-coordinate of obstacle 2

x_start = 3;        %x-coordinate of q_start
y_start = -0.8;     %y-coordinate of q_start
x_goal = 0;         %x-coordinate of q_goal
y_goal = 1;         %y-coordinate of q_goal


%Create Cspace.
figure(1)
[theta1, theta2] = meshgrid(-pi/2:0.02:3*pi/2,-pi/2:0.02:3*pi/2);

cond_1 = L_1.*sin(theta1)>-0.1;
cond_2 = (L_1.*cos(theta1) + 1./4.*L_2.*cos(theta1+theta2)-x_obst1).^2 + (L_1.*sin(theta1) + 1./4.*L_2.*sin(theta1+theta2)-y_obst1).^2 >(1./4.*L_2 + R_obstacle).^2;
cond_3 = (L_1.*cos(theta1) + 3./4.*L_2.*cos(theta1+theta2)-x_obst1).^2 + (L_1.*sin(theta1) + 3./4.*L_2.*sin(theta1+theta2)-y_obst1).^2 >(1./4.*L_2 + R_obstacle).^2;
cond_4 = (L_1.*cos(theta1) + 1./4.*L_2.*cos(theta1+theta2)-x_obst2).^2 + (L_1.*sin(theta1) + 1./4.*L_2.*sin(theta1+theta2)-y_obst2).^2 >(1./4.*L_2 + R_obstacle).^2;
cond_5 = (L_1.*cos(theta1) + 3./4.*L_2 *cos(theta1+theta2)-x_obst2).^2 + (L_1.*sin(theta1) + 3./4.*L_2.*sin(theta1+theta2)-y_obst2).^2 >(1./4.*L_2 + R_obstacle).^2;

conditions = cond_1&cond_2&cond_3&cond_4&cond_5;    %combines the conditions to create cspace
cspace = conditions;

%Create a state space.
ss = stateSpaceSE2;
%Create an occupanyMap-based state validator using the created state space.
sv = validatorOccupancyMap(ss);

%Create an occupany map from an example map and set map resolution as 10 cells/meter.
map = occupancyMap(flip(1-cspace),52.5);
sv.Map = map;
figure(1)
show(map); 
title('Free and Obstacle Space');

%Set validation distance for the validator.
sv.ValidationDistance = pi/50;
%Update state space bounds to be the same as map limits.
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]];
    
%Set the start and goal states.
start = [x_start+1.5, y_start+1.5, 0];
goal = [x_goal+1.5, y_goal+1.5, 0];

%RRT algortim using plannerRRT

%Create the path planner and increase max connection distance.
planner = plannerRRT(ss,sv);
planner.MaxIterations = 6000;
planner.MaxConnectionDistance = pi/50;
%Plan a path with default settings.
[pthObj,solnInfo] = plan(planner,start,goal);
%Visualize the results.
figure(2)
show(map); 
title('RRT');
start_text = text(start(1),start(2),'Start'); 
start_text.FontSize = 15; 
start_text.Color = 'r';
goal_text = text(goal(1),goal(2),'Goal'); 
goal_text.FontSize = 15; 
goal_text.Color = 'r';

hold on
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-'); % tree expansion
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2) % draw path

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%RRT* algortim using plannerRRTstar

%Create RRT* path planner and allow further optimization after goal is reached. Reduce the maximum iterations and increase the maximum connection distance.
planner2 = plannerRRTStar(ss,sv);
planner2.ContinueAfterGoalReached = true; % Allow further optimization
planner2.MaxIterations = 6000; % Number of max iterations
planner2.MaxConnectionDistance = pi/50; % Increase max connection distance

%Plan a path with default settings.
[pthObj,solnInfo] = plan(planner2,start,goal);

%Visualize the results.
figure(3)
map.show;
title('RRT*')
start_text = text(start(1),start(2),'Start'); 
start_text.FontSize = 15; 
start_text.Color = 'r';
goal_text = text(goal(1),goal(2),'Goal'); 
goal_text.FontSize = 15; 
goal_text.Color = 'r';

hold on
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-') % Tree expansion
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2) % Draw path



