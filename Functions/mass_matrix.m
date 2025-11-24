function [mass_matrix] = mass_matrix(m_pl, J_pl, J_rob, m_rob)
    % vector is [xp yp zp phi xg1 yg1 zg1 alpha xg2 yg2 zg2 psi xg3 yg3 zg3
    % theta]

    mass_matrix = zeros(16);

    % Link masses
    m1 = m_rob(1);
    m2 = m_rob(2);
    m3 = m_rob(3);

    % Link Inertia Tensors (may change to vectors
    J1 = J_rob(:,:,1);
    J2 = J_rob(:,:,2);
    J3 = J_rob(:,:,3);
    
    % Payload mass contribution
    mass_matrix(1:3, 1:3) = m_pl*eye(3);
    
    % Link mass contributions
    mass_matrix(5:7, 5:7) = m1*eye(3);
    mass_matrix(9:11, 9:11) = m2*eye(3);
    mass_matrix(13:15, 13:15) = m3*eye(3);

    % Link inertia contributions (diagonal)
    mass_matrix(4,4) = J_pl(3,3) + J3(3,3);  % phi
    mass_matrix(8,8) = J1(3,3);         % alpha
    mass_matrix(12,12) = J2(3,3);       % psi
    mass_matrix(16, 16) = J_pl(1,1) + J1(1,1) + J2(1,1) + J3(1,1);
    
    J_tot = J_pl(1,1) + J1(1,1) + J2(1,1) + J3(1,1);
    % Simplified Case
    mass_matrix = diag([m_pl, m_pl, m_pl, J_pl(3,3) + J3(3,3), m1, m1, m1, J1(3,3), m2, m2, m2, J2(3,3), m3, m3, m3, J_tot]);

end
