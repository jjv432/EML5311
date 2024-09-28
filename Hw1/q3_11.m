clc; clear all; close all; format compact;


clc; clear all; close all; format compact

% Jack Vranicar
% jjv20@fsu.edu
%% HW1, Q 3.10

%{

    It seems like the force is being applied to the inner top box, not the
    outer box. =(

%}

%% Parameters
m_3 = 1; % kg
m_2 = 2; %kg
m_1 = 1; %kg

f_v3 = 1; % N*s/m
f_v2 = 1; % N*s/m
f_v1 = 1; % N*s/m

k_2 = 1; % N/m
k_1 = 1; % N/m


%% A, B, C, D Matrices
A = [
    0 , 1 , 0 , 0 , 0 , 0;
    -k_2/m_1 , (-f_v3 - f_v2)/(m_2) , 0 , (f_v2)/(m_2) , (k_2)/(m_2) , (f_v3)/(m_2);
    0 , 0 , 0 , 1 , 0 , 0;
    0 , (f_v2)/(m_2) , (k_1)/(m_2) , (-f_v2 - f_v1)/(m_2) , (k_1)/(m_2) , (f_v2)/(m_2);
    0 , 0 , 0 , 0 , 0 , 1;
    (k_2)/(m_3) , (f_v3)/(m_3) , (-k_1)/(m_3) , (f_v1)/(m_3) , (k_1 - k_2)/(m_3) , (-f_v3 - f_v1)/(m_3);
    ];

B = [
0;
0;
0;
0;
0;
(1)/(m_3);
];

C = [

1 , 0 , 0 , 0 , 0 , 0 ;
0 , 0 , 1 , 0 , 0 , 0 ;
0 , 0 , 0 , 0 , 1 , 0 ;


];

D = [
    0;
    0;
    0;
    ];

%% Simulation

T = ss(A, B, C, D);

% Z = initial(T, [1 0 0 0 0 0], 10);
% times = linspace(0, 10, numel(Z(:,1)));

t = linspace(0, 10, 200);
f = 2*cos(t);

Z = lsim(T, f, t);

lsim(T,f,t);

%% Boxes
figure();
vid = VideoWriter("Question3_11.avi");
vid.FrameRate = 20;
vid.Quality = 85;
open(vid);

for i = 1:size(Z, 1)
    pause(.01);
    boxPlotter(Z(i,1), Z(i,2), Z(i,3));
    writeVideo(vid, getframe(gcf));
end
close(vid)
%% Springs



