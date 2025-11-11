function Q= PROinv(S, L, sol)
    % inverse kinematic: 4DOF robot
    % sol variable is just to understand which beta we take
    Q = zeros(4,1);
    x = S(1); 
    y = S(2);
    z = S(3);
    phi = S(4);

    l1 = L(1); 
    l2 = L(2);
    l3 = L(3);

    % theta
    Q(4) = atan2(y,x);

    % aux coords
    xb = x - l3*cos(phi)*cos(Q(4));
    yb = y - l3*sin(phi)*cos(Q(4));
    zb = z - l3*sin(phi);
    
    % beta
    cb = (xb^2 + yb^2 + zb^2 - l1^2 - l2^2)/(2*l1*l2);
    sb = sqrt(1-cb^2);
    beta = atan2(sb,cb);
    if (sol > 0)
     Q(2) = beta;
    else
     Q(2) = - beta;
    end

    % alpha
    Q(1) = atan2(zb,sqrt(xb^2 + yb^2))- atan2(l2*sin(Q(2)), l1+l2*cos(Q(2)));

    % gamma
    Q(3) = phi - Q(1) - Q(2);
end