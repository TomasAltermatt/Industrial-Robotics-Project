function Q = PROinv2(S, L, sol)
    % Inverse kinematics for 4DOF robot
    % S = [x; y; z; phi]  -> desired pose
    % L = [l1 l2 l3]      -> link lengths
    % sol = +1 or -1      -> elbow up / elbow down solution

    Q = zeros(4,1);

    % Extract variables
    x = S(1); 
    y = S(2);
    z = S(3);
    phi = S(4);

    l1 = L(1); 
    l2 = L(2);
    l3 = L(3);

    % --- 1. Base Angle (theta) ---
    Q(4) = atan2(y,x);

    % --- 2. Proyection in plane XZ ---
    r = sqrt(x^2 + y^2);   % Distance in XY
    zp = z;
    
    xp = r - l3*cos(phi);
    zp = zp - l3*sin(phi);


    % --- 4. Calculate Beta Angle(Joint 2) ---
    D = (xp^2 + zp^2 - l1^2 - l2^2) / (2*l1*l2);
    D = max(min(D, 1), -1);               % Numeric Saturation
    beta = atan2(sol*sqrt(1 - D^2), D);
    Q(2) = beta;

    % --- 5. Calculates Alpha Angle (Joint 1) ---
    Q(1) = atan2(zp, xp) - atan2(l2*sin(Q(2)), l1 + l2*cos(Q(2)));

    % --- 6. Calculates Gamma (Joint 3) ---
    Q(3) = phi - Q(1) - Q(2);

end
