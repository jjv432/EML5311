clc; 
clear all
close all
format compact

rho = 1055;
g = 9.81;
L = 0.43;
volume = .1 * .1* L;
m = rho * volume;
I = (1/3) * m * L^2;

open_loop = tf([1], [I, 0, m*g]);

closed_loop = tf([1 5], [I, 0, m*g*L/2]);

% rlocus(closed_loop)

%% Plotting

x_vals = [0 0 1 1];
y_vals = [-.5 .5 .5 -.5];
Coords = [x_vals; y_vals];

thetas = step(closed_loop);

RotMatrix = [cos(thetas), -sin(thetas); sin(thetas) cos(thetas)];

figure();
for i = 1:length(thetas)
    rot_coords = RotMatrix * Coords;
    h1 = fill(rot_coords(1, :), rot_coords(2, :), 'k');
    drawnow;
    pause(.1);
    delete(h1);
end








