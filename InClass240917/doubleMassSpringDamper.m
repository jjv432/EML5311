clc
clear all
close all

m1 = 2; m2 = 1; k1 = 10; k2 = 20; c1 = 1;

A = [0 1 0 0; -(k1 +k2)/m1 -c1/m1 k2/m1 0; 0 0 0 1; k2/m2 0 -k2/m2 0];
B = [0 0; 1/m1 0; 0 0; 0 1/m2];
C = [1 0 0 0; 0 0 1 0];
D = [0 0; 0 0];
G = ss(A, B, C, D);

initial(G, [0 0 1 0], 10);