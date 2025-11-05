function S= PROdir(Q,L)
    % direct kinematics: Project 4DOF robot
    l1 = L(1);
    l2 = L(2);
    l3 = L(3);

    S= zeros(4,1);
    S(1) = (l1*cos(Q(1)) + l2*cos(Q(1) + Q(2)) + l3*cos(Q(1) + Q(2) + Q(3)))*cos(Q(4));
    S(2) = (l1*cos(Q(1)) + l2*cos(Q(1) + Q(2)) + l3*cos(Q(1) + Q(2) + Q(3)))*sin(Q(4));
    S(3) = l1*sin(Q(1)) + l2*sin(Q(1) + Q(2)) + l3*sin(Q(1) + Q(2) + Q(3));
    S(4) = Q(1) + Q(2) + Q(3);
end