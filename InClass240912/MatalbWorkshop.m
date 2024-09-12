%% Example: Pendulum under gravity

%{
    Input is tau(t), output is theta(t), mass m, length L
%}

%% Simulating w/ transfer fns

%G = tf([1 2 3], [1 3 5 7]); %example

%% Beginning of Dr. Hubicki typing

clc; clear; close all;

m = 1; %kg
g = 9.8; %m/s/s
L = 1; %m

%Creating TF

G = tf([1], [m*L^2, 0, m*g*L]);

%Step input
%step(G, 10) %step response of G for 10 seconds

option = RespConfig;

option.Amplitude = 5;

step(G,10, option);


% using lsim

figure()
t = [0:.01:10];
u = t; %Ramp function
lsim(G, u, t); %becomes less accurate b/c assuming small angle

figure()
pzmap(G) %pole map of G
rlocus(G) %root-locus of G

% Other way to build TF (if you know zeros)
G_2 = zpk([], [sqrt(9.8)*i, -sqrt(9.8)*i], 1)

%% ODE 45

%ss command is for state space

A = [0 1; -g/L 0];
B = [0; 1/(m*L^2)];
C = [1 0];
D = 0;

G_3 = ss(A,B,C,D);
figure()
step(G_3, 10)
figure()
initial(G_3, [1 0], 10)%don't need 0 I.C.'s for ss 
%The vector [1 0] is the order of the states in the state vector (theta,
%then theta_dot

%% ODE 45
%Going to undo the linear approximation now, because we can simulate
%non-linear systems w / ODE45

[tout, xout] = ode45(@dynamics, [0 10], [1 0]);
%order of inputs to ode45: ode function, timespan, initial conditions
figure()
plot(tout, xout(:,1))