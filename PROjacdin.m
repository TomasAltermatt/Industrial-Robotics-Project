function J=PROjacdin(Q,L)
    l1=L(1);
    l2=L(2);
    l3=L(3);

    g1 = L(4);
    g2 = L(5);
    g3 = L(6);

    alpha = Q(1);
    beta = Q(2);
    gamma = Q(3);
    theta = Q(4);

    a1 = alpha;
    a2 = alpha + beta; 
    a3 = alpha + beta + gamma;

    s = @(x) sin(x); c = @(x) cos(x);
    
    % Vector of type
    % [xp, yp, zp, phi, xg1, yg1, zg1, alpha, xg2, yg2, zg2, psi, xg3, yg3, zg3, theta]
   J = [ 
    (-l1*s(a1)-l2*s(a2)-l3*s(a3))*c(theta), (-l2*s(a2)-l3*s(a3))*c(theta), -l3*s(a3)*c(theta), -(l1*c(a1)+l2*c(a2)+l3*c(a3))*s(theta);
    (-l1*s(a1)-l2*s(a2)-l3*s(a3))*s(theta), (-l2*s(a2)-l3*s(a3))*s(theta), -l3*s(a3)*s(theta),  (l1*c(a1)+l2*c(a2)+l3*c(a3))*c(theta);
     l1*c(a1)+l2*c(a2)+l3*c(a3),  l2*c(a2)+l3*c(a3),  l3*c(a3),  0;
     1, 1, 1, 0;
     -g1*s(a1)*c(theta), 0, 0, -g1*c(a1)*s(theta);
     -g1*s(a1)*s(theta), 0, 0,  g1*c(a1)*c(theta);
      g1*c(a1), 0, 0, 0;
      1, 0, 0, 0;
     -(l1*s(a1)+g2*s(a2))*c(theta), -g2*s(a2)*c(theta), 0, -(l1*c(a1)+g2*c(a2))*s(theta);
     -(l1*s(a1)+g2*s(a2))*s(theta), -g2*s(a2)*s(theta), 0,  (l1*c(a1)+g2*c(a2))*c(theta);
      l1*c(a1)+g2*c(a2),  g2*c(a2), 0, 0;
      1, 1, 0, 0;
     -(l1*s(a1)+l2*s(a2)+g3*s(a3))*c(theta), -(l2*s(a2)+g3*s(a3))*c(theta), -g3*s(a3)*c(theta), -(l1*c(a1)+l2*c(a2)+g3*c(a3))*s(theta);
     -(l1*s(a1)+l2*s(a2)+g3*s(a3))*s(theta), -(l2*s(a2)+g3*s(a3))*s(theta), -g3*s(a3)*s(theta),  (l1*c(a1)+l2*c(a2)+g3*c(a3))*c(theta);
      l1*c(a1)+l2*c(a2)+g3*c(a3),  l2*c(a2)+g3*c(a3),  g3*c(a3),  0;
      0, 0, 0, 1];
end