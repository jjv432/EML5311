clc; clear all; close all; format compact;

%

%% 3.14

A = [
    
    0 1 5 0;
    0 0 1 0;
    0 0 0 1;
    -7 -9 -2 -3;
];

B = [
    0;
    5;
    8;
    2;
    
];

C = [1 3 6 6];

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


fprintf("%s\n ---------------------------------------------- \n %s\n", numStr, demStr)