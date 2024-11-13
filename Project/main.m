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

T = tf([1], [I, 0, m*g]);

resp = step(T, 10);

%% Animate Rectangle

xvals = w*[-.5 -.5 .5 .5];
yvals = l*[0 1 1 0];

coords = [xvals; yvals];
figure();
thetas = linspace(0, 2*pi, 50);
for i = 1:length(thetas)

    theta = thetas(i);
    rotMatrix = [sin(theta), cos(theta); -cos(theta), sin(theta)];

    rotCoords = rotMatrix* coords;

    h1 = fill(rotCoords(1,:), rotCoords(2,:), 'k');
    axis equal
    axis([-1 1 -1 1])
    pause(.1)
    delete(h1)
end
