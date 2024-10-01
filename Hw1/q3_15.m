clc; clear all; close all; format compact;

%

%% 3.15

M = 2.4;
m = 0.23;
l = 0.36;
g = 9.81;

A = [
    
    0 1 0 0;
    (M + m)*g/(M*l) 0 0 0;
    0 0 0 1;
    -m*g/M 0 0 0;
];

B = [
    0;
    -1/M*l;
    0;
    1/M;
    
];

C = [l 0 1 0]; %first one is L, not 1!

D = [0];

[af, bf] = ss2tf(A, B, C, D);


numStr = " ";
initPow = numel(af);
for i = 1:numel(af)
   
    numStr = strcat(numStr, num2str(af(i)),"*s^", num2str(initPow -1));
    if i < numel(af)
        numStr = strcat(numStr, " + ");
    end

    initPow = initPow -1;
end

demStr = " ";
initPow = numel(bf);
for i = 1:numel(bf)
   
    demStr = strcat(demStr, num2str(bf(i)),"*s^", num2str(initPow -1));
    if i < numel(bf)
        demStr = strcat(demStr, " + ");
    end

    initPow = initPow -1;
end


fprintf("%s\n", numStr)

for i = 1:length(char(numStr))
    fprintf("-");
end

fprintf("\n%s\n", demStr)