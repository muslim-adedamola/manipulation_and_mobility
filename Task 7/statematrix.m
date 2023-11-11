function [xx1, xx2, uu1, uu2] = state_matrixs(x_star, dt)
x1 = []; x2 = [];
u1 = []; u2 = [];

x1(1) = x_star(1);
x2(1) = x_star(2);
u1(1) = x_star(3);
u2(1) = x_star(4);

x1(2) = x1(1)+ x_star(3)*dt
x2(2) = x2(1) + x_star(4)*dt

k = 2;
for i=5:2:size(x_star,1)
    x1(k+1) = x1(k) + x_star(i)*dt;
    x2(k+1) = x2(k) + x_star(i+1)*dt;
    u1(k) = x_star(i);
    u2(k) = x_star(i+1);
    k = k + 1;
end
u1(k) = u1(k-1);
u2(k) = u2(k-1);

xx1 = x1; xx2 = x2;
uu1 = u1; uu2 = u2;
end