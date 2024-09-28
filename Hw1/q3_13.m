clc; clear all; close all; format compact;

b = [8 10];
a = [1 5 1 5 13];

[A,B,C,D] = tf2ss(b,a)