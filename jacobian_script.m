clear all; close all; clc;
addpath('Functions')

syms alpha beta gamma theta l1 l2 l3 

% --- Precompute angles ---
    a1 = alpha;
    a2 = alpha + beta;
    a3 = alpha + beta + gamma;
    
    % --- Intermediate geometric terms ---
    r = l1*cos(a1) + l2*cos(a2) + l3*cos(a3);
    s = l1*sin(a1) + l2*sin(a2) + l3*sin(a3);

J = [ ...
        -s*cos(theta),  -(l2*sin(a2) + l3*sin(a3))*cos(theta),  -l3*sin(a3)*cos(theta),  -r*sin(theta);
        -s*sin(theta),  -(l2*sin(a2) + l3*sin(a3))*sin(theta),  -l3*sin(a3)*sin(theta),   r*cos(theta);
         r,              (l2*cos(a2) + l3*cos(a3)),              l3*cos(a3),              0;
         1,                          1,                                1,                 0 ];

detJ_simp = simplify(det(J));

detJfunc = matlabFunction(detJ_simp);


