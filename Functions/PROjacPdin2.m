function Jp = PROjacPdin2(Q, Qp, L)
% Calculates the time derivative of the Jacobian matrix (J_dot or Jp).
%
% INPUTS:
%   Q  = [alpha; beta; gamma; theta] - Vector of joint positions
%   Qp = [alpha_dot; beta_dot; gamma_dot; theta_dot] - Vector of joint velocities
%   L  = [l1; l2; l3; g1; g2; g3] - Vector of constant parameters

    % Unpack parameters
    l1 = L(1);
    l2 = L(2);
    l3 = L(3);

    g1 = L(4);
    g2 = L(5);
    g3 = L(6);

    % Unpack joint positions
    alpha = Q(1);
    beta  = Q(2);
    gamma = Q(3);
    theta = Q(4);

    % Unpack joint velocities
    ap = Qp(1); % alpha_dot
    bp = Qp(2); % beta_dot
    gp = Qp(3); % gamma_dot
    tp = Qp(4); % theta_dot

    % Intermediate angle positions
    a1 = alpha;
    a2 = alpha + beta; 
    a3 = alpha + beta + gamma;

    % Intermediate angle velocities (time derivatives of a1, a2, a3)
    a1p = ap;
    a2p = ap + bp;
    a3p = ap + bp + gp;

    % Anonymous functions for sine and cosine
    s = @(x) sin(x); 
    c = @(x) cos(x);
    
    % Pre-calculate trig values for positions
    s1 = s(a1); c1 = c(a1);
    s2 = s(a2); c2 = c(a2);
    s3 = s(a3); c3 = c(a3);
    st = s(theta); ct = c(theta);

    % --- Define Intermediate Terms (from original J) ---
    % These are the "position" level terms
    T1 = l1*c1 + l2*c2 + l3*c3;
    T2 = l1*s1 + l2*s2 + l3*s3;
    T3 = l2*c2 + l3*c3;
    T4 = l2*s2 + l3*s3;
    T5 = l3*c3;
    T6 = l3*s3;
    
    G1c = g1*c1;
    G1s = g1*s1;
    
    g2c = g2*c2;
    g2s = g2*s2;
    
    g3c = g3*c3;
    g3s = g3*s3;

    T_g2c = l1*c1 + g2*c2;
    T_g2s = l1*s1 + g2*s2;
    
    T_g2sc = l2*c2 + g3*c3; % Term from J(15,2)
    T_g2ss = l2*s2 + g3*s3; % Term from J(13,2)
    
    T_g3c = l1*c1 + l2*c2 + g3*c3;
    T_g3s = l1*s1 + l2*s2 + g3*s3;

    % --- Define Derivatives of Intermediate Terms ---
    % These are the "velocity" level terms, d/dt(Term)
    T1p = -l1*s1*a1p - l2*s2*a2p - l3*s3*a3p;
    T2p =  l1*c1*a1p + l2*c2*a2p + l3*c3*a3p;
    T3p = -l2*s2*a2p - l3*s3*a3p;
    T4p =  l2*c2*a2p + l3*c3*a3p;
    T5p = -l3*s3*a3p;
    T6p =  l3*c3*a3p;

    G1cp = -g1*s1*a1p;
    G1sp =  g1*c1*a1p;

    g2cp = -g2*s2*a2p;
    g2sp =  g2*c2*a2p;

    g3cp = -g3*s3*a3p;
    g3sp =  g3*c3*a3p;

    T_g2cp = -l1*s1*a1p - g2*s2*a2p;
    T_g2sp =  l1*c1*a1p + g2*c2*a2p;
    
    T_g2scp = -l2*s2*a2p - g3*s3*a3p;
    T_g2ssp =  l2*c2*a2p + g3*c3*a3p;

    T_g3cp = -l1*s1*a1p - l2*s2*a2p - g3*s3*a3p;
    T_g3sp =  l1*c1*a1p + l2*c2*a2p + g3*c3*a3p;

    % Derivatives of theta terms
    ctp = -st * tp; % d/dt(cos(theta))
    stp =  ct * tp; % d/dt(sin(theta))

    % --- Construct the Jp matrix ---
    % Apply product rule d/dt(A*B) = (A_dot * B) + (A * B_dot) to each element
    
    Jp = [ ...
    (-T2p*ct - T2*ctp), (-T4p*ct - T4*ctp), (-T6p*ct - T6*ctp), (-T1p*st - T1*stp);
    (-T2p*st - T2*stp), (-T4p*st - T4*stp), (-T6p*st - T6*stp), ( T1p*ct + T1*ctp);
     T1p,                T3p,                T5p,                0;
     0,                  0,                  0,                  0;
    (-G1sp*ct - G1s*ctp), 0,                  0,                 (-G1cp*st - G1c*stp);
    (-G1sp*st - G1s*stp), 0,                  0,                 ( G1cp*ct + G1c*ctp);
     G1cp,               0,                  0,                  0;
     0,                  0,                  0,                  0;
    (-T_g2sp*ct - T_g2s*ctp), (-g2sp*ct - g2s*ctp), 0,            (-T_g2cp*st - T_g2c*stp);
    (-T_g2sp*st - T_g2s*stp), (-g2sp*st - g2s*stp), 0,            ( T_g2cp*ct + T_g2c*ctp);
     T_g2cp,                  g2cp,               0,                  0;
     0,                       0,                  0,                  0;
    (-T_g3sp*ct - T_g3s*ctp), (-T_g2ssp*ct - T_g2ss*ctp), (-g3sp*ct - g3s*ctp), (-T_g3cp*st - T_g3c*stp);
    (-T_g3sp*st - T_g3s*stp), (-T_g2ssp*st - T_g2ss*stp), (-g3sp*st - g3s*stp), ( T_g3cp*ct + T_g3c*ctp);
     T_g3cp,                  T_g2scp,                 g3cp,                0;
     0,                       0,                       0,                  0];
end