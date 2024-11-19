% J Vranicar 11/10/24

clc;
clear all;
close all;
format compact;

%% Problem 20

T = tf([16], [1 3 16]);

[wn, zeta] = damp(T);

wn = wn(1)
zeta = zeta(1)

Ts = 4 / (zeta*wn)

Tp = pi/(wn * sqrt(1 - zeta^2))

% Interpolating from rise time table
Tr_norm = ((zeta - 0.3) / (.4 - .3)) * (1.463 - 1.321) + 1.321;
Tr = Tr_norm/wn

perc_OS = exp(-(zeta*pi/sqrt(1-zeta^2))) * 100
