% J Vranicar 11/10/24

clc;
clear all;
close all;
format compact;

%% Problem 18

%% Part C

Tc = tf([10 70], [1 30 200]);

[wn, zeta_c] = damp(Tc);
zeta_c = zeta_c(1)
wn_c = wn(1)

%% Part D

Td = tf([20], [1 6 144]);

[wn_d, zeta_d] = damp(Td);
zeta_d = zeta_d(1)
wn_d = wn_d(1)

%% Part E

Te = tf([1 2], [1 0 9]);

[wn_e, zeta_e] = damp(Te);
zeta_e = zeta_e(1)
wn_e = wn_e(1)
%% Part F

Tf = tf([1 5], [1 20 100]);

[wn_f, zeta_f] = damp(Tf);
zeta_f = zeta_f(1)
wn_f = wn_f(1)