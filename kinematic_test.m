clear
close all
clc


l1 = 50; l2 =50; l3 = 20; % Length of the links
L=[l1; l2; l3]';

% End Effector Position
S = [50; 25; 35; pi/6];

Q1=PROinv(S,L,1);  % first solution (beta >0)
Q2=PROinv(S,L,-1); % second solution (beta <0)

S1=PROdir(Q1, L) % check 1 sol
S2=PROdir(Q2, L) % check 2 sol

PlotPRO(Q1, L);
PlotPRO(Q2, L);

