% J Vranicar 11/10/24

clc; 
clear all;
close all;
format compact;

%% Problem 9

T = tf([1 2 2], [1 6 4 7 2]);

pole(T)