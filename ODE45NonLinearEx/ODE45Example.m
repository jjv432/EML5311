clc
clearvars
close all
format compact

x0 = [0; 1];

[tout, xout] = ode45(@dynamics, [0 10], x0); % fn name, time span, initial state

plot(tout, xout(:,2), 'k-')
xlabel('Time')
ylabel("Resp")


function [dx] = dynamics(t, x)

% inputs are time and state
dx = [0;0];

dx(1) = 1-x(2); %derivative of the first state is equal to the second
dx(2) = -2*x(2) + 2*(1-x(2)) + 8*t - 8*x(1);


end