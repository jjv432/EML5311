clc; clear all; close all; format compact;

% Make sure to adjust L, rn it's just 1

end_theta = 0;
end_theta_dot = 0;
m = 10; 
L = 1;
g = 9.81;
I = m*L^2/3;

A = [0, 1; -m*g*cos(end_theta)/(2*I) 0];
B = [0; 1/I];
C = [1 0; 0 1];
D = [0; 0];

G = ss(A, B, C, D);
t = 0:.001:.005;

Kp = 0;
theta_des = pi/2;

positionHistory = [];

% Create a loop to update theta at small increments
for i = 1:1000
    u = Kp*(theta_des - end_theta)*ones(length(t), 1);

% Simulate the system as linear for a short amount of time
simValues = lsim(G, u, t, [end_theta end_theta_dot]);
thetas = simValues(:, 1);
end_theta = thetas(end);
end_theta_dot = simValues(end, 2);
positionHistory = [positionHistory; thetas];

A(2) = -m*g*cos(end_theta)/(2*I);
G = ss(A, B, C, D);


end

close all
% figure()
% plot(1:length(positionHistory), positionHistory)


%% Animate Rectangle
w = 1/3;
l = 1;

xvals = l*[0 0 1 1];
yvals = w*[-.5 .5 .5 -.5];

coords = [xvals; yvals];
figure();
thetas = positionHistory;

for i = 1:50:length(thetas)

    theta = thetas(i);
    rotMatrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];

    rotCoords = rotMatrix* coords;

    h1 = fill(rotCoords(1,:), rotCoords(2,:), 'k');
    axis equal
    axis([-2 2 -2 2])
    drawnow;
    delete(h1)
end