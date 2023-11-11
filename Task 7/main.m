% Clear workspace
clc;
clear;
close all;

% Define system parameters
L_1 = 0.5;
L_2 = 0.4;
R = 0.2;
Ox1 = -0.6;
Oy1 = 0.7;
Ox2 = 0.6;
Oy2 = 0.7;

% Optimal Control Problem (OCP) setup
syms k h1 h2 h3 h4 h5 h6 h7 h8 h9 h10;

N = 25;
dt = 5 / N;
t = 0:1:N;

% Generate symbolic vectors
x1 = transpose(sym('x1_', [1 N + 1]));
x2 = transpose(sym('x2_', [1 N + 1]));
u1 = transpose(sym('u1_', [1 N]));
u2 = transpose(sym('u2_', [1 N]));

func = 0;

for i=1:N
    %the cost function
    func = func + dt*(u1(i)^2 + u2(i)^2);
    
    % here we calculate dynamics of the system in symbolic form   
    x1(i+1) = x1(i) + dt*u1(i); 
    x2(i+1) = x2(i) + dt*u2(i);

    h1(i) = -L_1*sin(x1(i)) - 0.1;
    h2(i) = -L_1*sin(x1(i)) - L_2*sin(x1(i)+x2(i)) - 0.1;
    
    h3(i) = -((L_1*cos(x1(i))+L_2/6.*cos(x1(i)+x2(i)))-Ox1)^2 - ((L_1*sin(x1(i))+L_2/6*sin(x1(i)+x2(i)))-Oy1)^2 + (L_2/6 + R)^2;
    h4(i) = -((L_1*cos(x1(i))+L_2/2*cos(x1(i)+x2(i)))-Ox1)^2 - ((L_1*sin(x1(i))+L_2/2*sin(x1(i)+x2(i)))-Oy1)^2 + (L_2/6 + R)^2;
    h5(i) = -((L_1*cos(x1(i))+5/6*L_2*cos(x1(i)+x2(i)))-Ox1)^2 - ((L_1*sin(x1(i))+5/6*L_2*sin(x1(i)+x2(i)))-Oy1)^2 + (L_2/6 + R)^2;
    
    h6(i) = -((L_1*cos(x1(i))+L_2/6*cos(x1(i)+x2(i)))-Ox2)^2 - ((L_1*sin(x1(i))+L_2/6*sin(x1(i)+x2(i)))-Oy2)^2 + (L_2/6 + R)^2;
    h7(i) = -((L_1*cos(x1(i))+L_2/2*cos(x1(i)+x2(i)))-Ox2)^2 - ((L_1*sin(x1(i))+L_2/2*sin(x1(i)+x2(i)))-Oy2)^2 + (L_2/6 + R)^2;
    h8(i) = -((L_1*cos(x1(i))+5/6*L_2*cos(x1(i)+x2(i)))-Ox2)^2 - ((L_1*sin(x1(i))+5/6*L_2*sin(x1(i)+x2(i)))-Oy2)^2 + (L_2/6 + R)^2;
    
    h9(i) = x2(i)-pi/2;
    h10(i) = -x2(i)-pi/2;
end

% Define the discrete function
discrete = func;

% Define the nonlinear inequality constraints as vectors
c = [h9'; h10'; h1'; h2'; h3'; h4'; h5'; h6'; h7'; h8'];
% Define the vector of equality constraints
ceq = [x1(1); (x2(1) - (pi / 6)); (x1(N + 1) - (5 * pi / 6)); x2(N + 1)];

% Initialize empty matrices for equality and inequality constraints
A = [];
b = [];
C = [];
d = [];

% Define no box constraints (lower and upper bounds)
lb = [];
ub = [];

% Define the vector of decision variables
v = [x1(1); x2(1); u1; u2];

% Define the initial point
%x0 = [0.00; 0.00; rand(2 * N, 1)];

x0 = [0
0
0.0323355534464184
0.557066829681192
0.719801554892191
0.110408048956958
0.216647376280279
0.811020215616459
0.138661882869267
0.881899200733092
0.923556061690306
0.0127555628361417
0.377159253042995
0.167811688977599
0.540222795744860
0.101662434927830
0.0392677458897454
0.933229111773545
0.971591882864217
0.360928020047589
0.644205403766746
0.0679473020936823
0.207911996415632
0.0396038467757357
0.469359313223853
0.150096722662633
0.991306921184190
0.427062347759695
0.955372102971021
0.724247033520084
0.580891712363774
0.540257907420780
0.705441191564081
0.00502888330112106
0.782515778836242
0.926859573385509
0.00829565739263971
0.824628342666394
0.767335868027880
0.997136895348686
0.227653072058219
0.919542206194465
0.641999305521780
0.105320182975915
0.268160913711683
0.763843764539584
0.805510155575541
0.104252997601917
0.469758776292843
0.219061890891174
0.922707906542724
0.320322632016148];


% Transform symbolic objective function into a function
discrete_func = matlabFunction(discrete, 'vars', {v});

% Transform symbolic constraints into functions
nonlcon = matlabFunction(c, ceq, 'vars', {v}, 'outputs', {'c', 'ceq'});

% Set optimization options
opts = optimset('Display', 'iter', 'Algorithm', 'interior-point', 'MaxFunEvals', 10000, 'MaxIter', 50000);

% Perform the constrained optimization
[x_star, f_star] = fmincon(discrete_func, x0, C, d, A, b, lb, ub, nonlcon, opts);

% Generate matrices x1, x2, u1, and u2 from the optimized solution
[x1_opt, x2_opt, u1_opt, u2_opt] = state_matrixs(x_star, dt);

% Plot results
figure(1);

% Time evolution of inputs plot
subplot(2, 1, 2);
hold on;
grid on;
title('Inputs plot');
xlabel('Time');
ylabel('u1 & u2');
plot(t, u1_opt);
plot(t, u2_opt);
legend({'u1', 'u2'});
hold off;

% Time evolution of system state plot
subplot(2, 1, 1);
hold on;
grid on;
title('System State plot');
xlabel('Time');
ylabel('x1 & x2');
plot(t, transpose(x1_opt));
plot(t, transpose(x2_opt));
legend({'x1', 'x2'});
hold off;

figure(2)
theta1 = linspace(-0.5*pi, 1.5*pi); % rotation limits of the first joint [0, 2*pi]
theta2 = linspace(-0.5*pi,1.5*pi); % rotation limits of the second joint [-0.5*pi, 0.5*pi]

[X,Y] = meshgrid(theta1,theta2);    % generation of the grid
Z = zeros([100, 100]);  % boolean map initialization (0 for free space, 1 for obstacle space), initially all 0

% check conditions for obstacle for each possible configuration:
for j = 1:100
    for i = 1:100
        if is_obstacle(theta1(i), theta2(j))
            Z(j, i) = 1;
        end
    end
end

contourf(1-Y, X, Z,[1,1], 'ShowText','on') % generate map from Z on top of the grid [X, Y]
hold on
plot(double(x2_opt), double(x1_opt), '-.','Color','red')
hold off