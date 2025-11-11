function J = PROjac(Q, L)
    %   J : 4x4 Jacobian matrix (rows: [ẋ ẏ ż ϕ̇], cols: [α̇ β̇ γ̇ θ̇])

    l1=L(1);
    l2=L(2);
    l3=L(3);

    alpha = Q(1);
    beta = Q(2);
    gamma = Q(3);
    theta = Q(4);

    % --- Precompute angles ---
    a1 = alpha;
    a2 = alpha + beta;
    a3 = alpha + beta + gamma;
    
    % --- Intermediate geometric terms ---
    r = l1*cos(a1) + l2*cos(a2) + l3*cos(a3);
    s = l1*sin(a1) + l2*sin(a2) + l3*sin(a3);
    
    % --- Jacobian matrix ---
    J = [ ...
        -s*cos(theta),  -(l2*sin(a2) + l3*sin(a3))*cos(theta),  -l3*sin(a3)*cos(theta),  -r*sin(theta);
        -s*sin(theta),  -(l2*sin(a2) + l3*sin(a3))*sin(theta),  -l3*sin(a3)*sin(theta),   r*cos(theta);
         r,              (l2*cos(a2) + l3*cos(a3)),              l3*cos(a3),              0;
         1,                          1,                                1,                 0 ...
    ];

end