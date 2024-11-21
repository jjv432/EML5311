clc; clear all; close all; format compact;

end_theta = pi/4;
end_theta_dot = 0;
m = 1; 
g = 9.81;
I = 5;

A = [0, 1; m*g*cos(end_theta)/(2*I) 0];
B = [0; 1/I];
C = [1 0; 0 1];
D = [0; 0];

G = ss(A, B, C, D);

positionHistory = [];

% Create a loop to update theta at small increments
for i = 1:1000

% Simulate the system as linear for a short amount of time
simValues = initial(G, [end_theta end_theta_dot], .01);
thetas = simValues(:, 1);
end_theta = thetas(end);
end_theta_dot = simValues(end, 2);
positionHistory = [positionHistory; thetas];

A(2) = m*g*cos(end_theta)/(2*I);
G = ss(A, B, C, D);


end

close all
figure()
plot(1:length(positionHistory), positionHistory)