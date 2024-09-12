function [dx] = dynamics(t,x)


dx = zeros(2,1);
dx(1) = x(2);
dx(2) = -9.8*sin(x(1));

end