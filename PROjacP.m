function Jp = PROjacP(Q, Qd, L)
% PROjac_dot  Compute time derivative of the Jacobian matrix
%
% Inputs:
%   Q  : [alpha, beta, gamma, theta]
%   Qd : [alpha_dot, beta_dot, gamma_dot, theta_dot]
%   L  : [l1, l2, l3]
%
% Output:
%   Jp : 4x4 time derivative of Jacobian

l1 = L(1); l2 = L(2); l3 = L(3);
alpha = Q(1); beta = Q(2); gamma = Q(3); theta = Q(4);
alpha_p = Qd(1); beta_p = Qd(2); gamma_p = Qd(3); theta_p = Qd(4);

% --- Precompute composite angles ---
a1 = alpha;
a2 = alpha + beta;
a3 = alpha + beta + gamma;

a1d = alpha_p;
a2d = alpha_p + beta_p;
a3d = alpha_p + beta_p + gamma_p;

% --- Geometric terms ---
r = l1*cos(a1) + l2*cos(a2) + l3*cos(a3);
s = l1*sin(a1) + l2*sin(a2) + l3*sin(a3);

% --- Time derivatives of r and s ---
rd = -l1*sin(a1)*a1d - l2*sin(a2)*a2d - l3*sin(a3)*a3d;
sd =  l1*cos(a1)*a1d + l2*cos(a2)*a2d + l3*cos(a3)*a3d;

% --- Time derivative of Jacobian ---
Jp = zeros(4,4);

% Row 1 (ẋ)
Jp(1,1) = -sd*cos(theta) + s*sin(theta)*theta_p;
Jp(1,2) = -(l2*cos(a2)*a2d + l3*cos(a3)*a3d)*cos(theta) + (l2*sin(a2)+l3*sin(a3))*sin(theta)*theta_p;
Jp(1,3) = -(l3*cos(a3)*a3d)*cos(theta) + (l3*sin(a3))*sin(theta)*theta_p;
Jp(1,4) = -rd*sin(theta) - r*cos(theta)*theta_p;

% Row 2 (ẏ)
Jp(2,1) = -sd*sin(theta) - s*cos(theta)*theta_p;
Jp(2,2) = -(l2*cos(a2)*a2d + l3*cos(a3)*a3d)*sin(theta) - (l2*sin(a2)+l3*sin(a3))*cos(theta)*theta_p;
Jp(2,3) = -(l3*cos(a3)*a3d)*sin(theta) - (l3*sin(a3))*cos(theta)*theta_p;
Jp(2,4) =  rd*cos(theta) - r*sin(theta)*theta_p;

% Row 3 (ż)
Jp(3,1) = rd;
Jp(3,2) = -l2*sin(a2)*a2d - l3*sin(a3)*a3d;
Jp(3,3) = -l3*sin(a3)*a3d;
Jp(3,4) = 0;

% Row 4 (ϕ̇)
Jp(4,:) = [0 0 0 0];

end
