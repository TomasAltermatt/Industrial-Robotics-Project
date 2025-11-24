%% generate_PROjac_Standard.m
% Generates J and J_dot using STANDARD Right-Hand Rule rotations.
% This flips the sign of gy terms compared to your previous attempt.

clear; clc;

% 1. Define Symbolic Variables
syms l1 l2 l3 real
syms g1x g1y g1z g2x g2y g2z g3x g3y g3z real
syms alpha beta gamma theta real
syms alpha_dot beta_dot gamma_dot theta_dot real

% Group Inputs
Q = [alpha; beta; gamma; theta];
Qp = [alpha_dot; beta_dot; gamma_dot; theta_dot];
L_links = [l1; l2; l3; g1x; g1y; g1z; g2x; g2y; g2z; g3x; g3y; g3z];

% 2. Define Forward Kinematics (Standard Convention)
% --------------------------------------------------
a1 = alpha;
a2 = alpha + beta;
a3 = alpha + beta + gamma;

% --- Link 1 CoM ---
% U1: Horizontal projection along the radial line
% (Standard rotation: x' = x*cos - z*sin)
U1 = g1x*cos(a1) - g1z*sin(a1); 
Z1 = g1x*sin(a1) + g1z*cos(a1);

% X1/Y1: Standard Z-Rotation (x' = x*cos - y*sin)
% FLIPPED SIGN of g1y here compared to your previous code
X1 = U1*cos(theta) - g1y*sin(theta); 
Y1 = U1*sin(theta) + g1y*cos(theta);

Phi1 = a1;

% --- Link 2 CoM ---
U2 = l1*cos(a1) + g2x*cos(a2) - g2z*sin(a2);
Z2 = l1*sin(a1) + g2x*sin(a2) + g2z*cos(a2);

% FLIPPED SIGN of g2y here
X2 = U2*cos(theta) - g2y*sin(theta);
Y2 = U2*sin(theta) + g2y*cos(theta);

Phi2 = a2;

% --- Link 3 CoM ---
U3 = l1*cos(a1) + l2*cos(a2) + g3x*cos(a3) - g3z*sin(a3);
Z3 = l1*sin(a1) + l2*sin(a2) + g3x*sin(a3) + g3z*cos(a3);

% FLIPPED SIGN of g3y here
X3 = U3*cos(theta) - g3y*sin(theta);
Y3 = U3*sin(theta) + g3y*cos(theta);

Phi3 = theta; 

% --- End Effector (Assuming Tip is Centered) ---
U_ee = l1*cos(a1) + l2*cos(a2) + l3*cos(a3);
Z_ee = l1*sin(a1) + l2*sin(a2) + l3*sin(a3);
X_ee = U_ee*cos(theta);
Y_ee = U_ee*sin(theta);
Phi_ee = a3;

% 3. Calculate Matrices
% ---------------------
P_full = [X_ee; Y_ee; Z_ee; Phi_ee; ...
          X1;   Y1;   Z1;   Phi1;   ...
          X2;   Y2;   Z2;   Phi2;   ...
          X3;   Y3;   Z3;   Phi3];

% A. Jacobian J
fprintf('Calculating Jacobian J...\n');
J_sym = jacobian(P_full, Q);

% B. Time Derivative J_dot
fprintf('Calculating J_dot...\n');
J_dot_sym = sym(zeros(size(J_sym)));
for k = 1:4
    J_dot_sym = J_dot_sym + diff(J_sym, Q(k)) * Qp(k);
end

% 4. Generate Output Files
% ------------------------
fprintf('Writing PROjacdin.m ...\n');
matlabFunction(J_sym, 'File', 'PROjacdinV2', 'Vars', {Q, L_links}, 'Outputs', {'J'});

fprintf('Writing PROjacPdin.m ...\n');
matlabFunction(J_dot_sym, 'File', 'PROjacPdinV2', 'Vars', {Q, Qp, L_links}, 'Outputs', {'Jp'});

fprintf('Done. Use the new functions in your main script.\n');