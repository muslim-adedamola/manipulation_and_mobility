function [status] = is_obstacle(theta1, theta2)
    
    % properties of the system: 
    % length of the first link (l1) = 0.5 m
    % length of the second link (l2) = 0.4 m
    l1 = 0.5;
    l2 = 0.4;
    
    % properties of the environment:
    % center of the first circle obstacle (o1) = [0.6, 0.7] m
    % radius of the first circle obstacle (B1) = 0.2 m
    % center of the second circle obstacle (o2) = [-0.6, 0.7] m
    % radius of the first circle obstacle (B2) = 0.2 m
    % location of the wall (w): [..., -0.1] m
    o1 = [0.6, 0.7];
    B1 = 0.2;
    o2 = [-0.6, 0.7];
    B2 = 0.2;
    w_y = -0.1;
    
    % condition 1: the end effector is outside of the first circle obstacle
    cond1 = (l1*cos(theta1) + l2*cos(theta2+theta1) - o1(1))^2 + (l1*sin(theta1) + l2*sin(theta2+theta1) - o1(2))^2 < B1^2;
    
    cond1_2 = (l1*cos(theta1) + 2*l2*cos(theta2+theta1)/3 - o1(1))^2 + (l1*sin(theta1) + 2*l2*sin(theta2+theta1)/3 - o1(2))^2 < B1^2;

    cond1_3 = (l1*cos(theta1) + l2*cos(theta2+theta1)/3 - o1(1))^2 + (l1*sin(theta1) + l2*sin(theta2+theta1)/3 - o1(2))^2 < B1^2;

    % condition 2: the middle joint is outside of the first circle obstacle
    cond2 = (l1*cos(theta1) - o1(1))^2 + (l1*sin(theta1) - o1(2))^2 < B1^2;

    % condition 3: the end effector is outside of the second circle obstacle
    cond3 = (l1*cos(theta1) + l2*cos(theta2+theta1) - o2(1))^2 + (l1*sin(theta1) + l2*sin(theta2+theta1) - o2(2))^2 < B2^2;
    
    cond3_2 = (l1*cos(theta1) + 2*l2*cos(theta2+theta1)/3 - o2(1))^2 + (l1*sin(theta1) + 2*l2*sin(theta2+theta1)/3 - o2(2))^2 < B2^2;

    cond3_3 = (l1*cos(theta1) + l2*cos(theta2+theta1)/3 - o2(1))^2 + (l1*sin(theta1) + l2*sin(theta2+theta1)/3 - o2(2))^2 < B2^2;
    
    % condition 4: the middle joint is outside of the second circle obstacle
    cond4 = (l1*cos(theta1) - o2(1))^2 + (l1*sin(theta1) - o2(2))^2 < B2^2;
    
    % condition 5: the end effector is higher than the wall
    cond5 = (l1*sin(theta1) + l2*sin(theta2+theta1) < w_y);

    % condition 6: the middle joint is higher than the wall
    cond6 = (l1*sin(theta1) < w_y);

    % the configuration is in obstacle space if any of the conditions are satisfied
    status = cond1 || cond1_2 ||  cond1_3 || cond2 || cond3 || cond3_2 || cond3_3 || cond4 || cond5 || cond6;

end
