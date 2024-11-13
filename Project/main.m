clc;
clear all;
close all;
format compact;

%% Params
w = 0.1; % m
h = 0.1 ; %m
l = 0.43; %m 
rho = 1055; % kg/m/m/m
m = rho*w*h*l;

I = (1/3)*m*l^2;

g = 9.81; 

%% Transfer Function

T = tf([1], [I, 0, m*g*(l/2)]);

resp = step(T, 10);

%% State Spaces

A = [0 1; (m*g*l/(2*I)), 0];
B = [0; -1/I];
C = [1 0];
D = [0];

G = ss(A, B, C, D);

time = linspace(0, 5, 1000);
tau = 0 * ones(size(time, 2), 1);
ss_resp = lsim(G, tau, time, [pi; 0]);

%% ODE 45
% solving for theta or theta dd?
dynamics = @(tau, theta)(1/I) * (tau - m*g*sin(theta)*l/2);
[tout, xout] = ode45(dynamics, [0 10], [1 0]);
%order of inputs to ode45: ode function, timespan, initial conditions
figure()
plot(tout, xout(:,1))

%% Animate Rectangle

xvals = w*[-.5 -.5 .5 .5];
yvals = l*[0 1 1 0];

coords = [xvals; yvals];
figure();
thetas = linspace(0, 2*pi, 50);

for i = 1:length(xout)

    theta = xout(i);
    rotMatrix = [sin(theta), cos(theta); -cos(theta), sin(theta)];

    rotCoords = rotMatrix* coords;

    h1 = fill(rotCoords(1,:), rotCoords(2,:), 'k');
    axis equal
    axis([-1 1 -1 1])
    pause(.1)
    delete(h1)
end
