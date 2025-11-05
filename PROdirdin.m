function S = PROdirdin(Q,L)
    % direct kinematics: Project 4DOF robot
    l1 = L(1);
    l2 = L(2);
    l3 = L(3);

%     g1 = L(4);
%     g2 = L(5):
%     g3 = L(6):

    % Usually it should be known, but for now lets consider the halfs of
    % every length
    g1 = l1/2;
    g2 = l2/2;
    g3 = l3/2;

    S= zeros(16,1);

    S(1) = (l1*cos(Q(1)) + l2*cos(Q(1) + Q(2)) + l3*cos(Q(1) + Q(2) + Q(3)))*cos(Q(4));
    S(2) = (l1*cos(Q(1)) + l2*cos(Q(1) + Q(2)) + l3*cos(Q(1) + Q(2) + Q(3)))*sin(Q(4));
    S(3) = l1*sin(Q(1)) + l2*sin(Q(1) + Q(2)) + l3*sin(Q(1) + Q(2) + Q(3));
    S(4) = Q(1) + Q(2) + Q(3);

    S(5) = (g1*cos(Q(1)))*cos(Q(4));
    S(6) = (g1*cos(Q(1)))*sin(Q(4));
    S(7) = g1*sin(Q(1));
    S(8) = Q(1);

    S(9) = (l1*cos(Q(1)) + g2*cos(Q(1) + Q(2)))*cos(Q(4));
    S(10) = (l1*cos(Q(1)) + g2*cos(Q(1) + Q(2)))*sin(Q(4));
    S(11) = l1*sin(Q(1)) + g2*sin(Q(1) + Q(2));
    S(12) = Q(1) + Q(2);

    S(1) = (l1*cos(Q(1)) + l2*cos(Q(1) + Q(2)) + g3*cos(Q(1) + Q(2) + Q(3)))*cos(Q(4));
    S(2) = (l1*cos(Q(1)) + l2*cos(Q(1) + Q(2)) + g3*cos(Q(1) + Q(2) + Q(3)))*sin(Q(4));
    S(3) = l1*sin(Q(1)) + l2*sin(Q(1) + Q(2)) + g3*sin(Q(1) + Q(2) + Q(3));
    S(4) = Q(4);

end