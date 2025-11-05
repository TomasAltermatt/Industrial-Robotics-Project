function Jdot = PROjacPdin(Q, Qd, L)
    % Q  = [alpha; beta; gamma; theta]
    % Qd = [alpha_dot; beta_dot; gamma_dot; theta_dot]
    % L  = [l1 l2 l3 g1 g2 g3]
    %
    % Devuelve la derivada temporal del jacobiano (16x4)
    % correspondiente a la estructura definida en PROjacdin.

    % Parámetros
    l1 = L(1); l2 = L(2); l3 = L(3);
    g1 = L(4); g2 = L(5); g3 = L(6);

    % Variables
    alpha = Q(1); beta = Q(2); gamma = Q(3); theta = Q(4);
    adot  = Qd(1); bdot = Qd(2); gdot = Qd(3); tdot = Qd(4);

    % Atajos trigonométricos
    s = @(x) sin(x); c = @(x) cos(x);

    % Factores comunes para las tres secciones (l vs g)
    % Grupo l: [l1, l2, l3]
    A1 = l1*s(alpha) + l2*s(beta) + l3*s(gamma);
    C1 = l1*c(alpha) + l2*c(beta) + l3*c(gamma);
    A1dot = l1*c(alpha)*adot + l2*c(beta)*bdot + l3*c(gamma)*gdot;
    C1dot = -l1*s(alpha)*adot - l2*s(beta)*bdot - l3*s(gamma)*gdot;

    A2 = l2*s(beta) + l3*s(gamma);
    C2 = l2*c(beta) + l3*c(gamma);
    A2dot = l2*c(beta)*bdot + l3*c(gamma)*gdot;
    C2dot = -l2*s(beta)*bdot - l3*s(gamma)*gdot;

    A3 = l3*s(gamma);
    C3 = l3*c(gamma);
    A3dot = l3*c(gamma)*gdot;
    C3dot = -l3*s(gamma)*gdot;

    % Grupo g1: depende de alpha
    A1g = g1*s(alpha);   C1g = g1*c(alpha);
    A1gdot = g1*c(alpha)*adot;
    C1gdot = -g1*s(alpha)*adot;

    % Grupo g2: depende de alpha y beta
    A2g = l1*s(alpha) + g2*s(beta);
    C2g = l1*c(alpha) + g2*c(beta);
    A2gdot = l1*c(alpha)*adot + g2*c(beta)*bdot;
    C2gdot = -l1*s(alpha)*adot - g2*s(beta)*bdot;

    Ag2 = g2*s(beta);    Cg2 = g2*c(beta);
    Ag2dot = g2*c(beta)*bdot;
    Cg2dot = -g2*s(beta)*bdot;

    % Grupo g3: depende de alpha, beta y gamma
    A3g = l1*s(alpha) + l2*s(beta) + g3*s(gamma);
    C3g = l1*c(alpha) + l2*c(beta) + g3*c(gamma);
    A3gdot = l1*c(alpha)*adot + l2*c(beta)*bdot + g3*c(gamma)*gdot;
    C3gdot = -l1*s(alpha)*adot - l2*s(beta)*bdot - g3*s(gamma)*gdot;

    Ag3 = g3*s(gamma);   Cg3 = g3*c(gamma);
    Ag3dot = g3*c(gamma)*gdot;
    Cg3dot = -g3*s(gamma)*gdot;

    % Atajos de theta
    cth = c(theta); sth = s(theta);
    cthdot = -sth * tdot;
    sthdot =  cth * tdot;

    % Inicializa Jdot
    Jdot = zeros(16,4);

    % --- Bloque 1 (filas 1–4) con l1,l2,l3 ---
    % Fila 1: [ -A1*cth, -A2*cth, -A3*cth, -(C1)*sth ]
    Jdot(1,1) = -(A1dot*cth + A1*cthdot);
    Jdot(1,2) = -(A2dot*cth + A2*cthdot);
    Jdot(1,3) = -(A3dot*cth + A3*cthdot);
    Jdot(1,4) = -(C1dot*sth + C1*sthdot);

    % Fila 2: [ -A1*sth, -A2*sth, -A3*sth,  C1*cth ]
    Jdot(2,1) = -(A1dot*sth + A1*sthdot);
    Jdot(2,2) = -(A2dot*sth + A2*sthdot);
    Jdot(2,3) = -(A3dot*sth + A3*sthdot);
    Jdot(2,4) =  (C1dot*cth + C1*cthdot);

    % Fila 3: [ C1, C2, C3, 0 ]
    Jdot(3,1) = C1dot; Jdot(3,2) = C2dot; Jdot(3,3) = C3dot; Jdot(3,4) = 0;

    % Fila 4: [1, 1, 1, 0] -> constante, derivada cero
    Jdot(4,:) = [0 0 0 0]; %(implícito)

    % --- Bloque 2 (filas 5–8) con g1 ---
    % Fila 5: [ -g1*s(a)*cth, 0, 0, -g1*c(a)*sth ]
    Jdot(5,1) = -(A1gdot*cth + A1g*cthdot);
    Jdot(5,2) = 0;
    Jdot(5,3) = 0;
    Jdot(5,4) = -(C1gdot*sth + C1g*sthdot);

    % Fila 6: [ -g1*s(a)*sth, 0, 0,  g1*c(a)*cth ]
    Jdot(6,1) = -(A1gdot*sth + A1g*sthdot);
    Jdot(6,2) = 0;
    Jdot(6,3) = 0;
    Jdot(6,4) =  (C1gdot*cth + C1g*cthdot);

    % Fila 7: [ g1*c(a), 0, 0, 0 ]
    Jdot(7,1) = C1gdot; Jdot(7,2) = 0; Jdot(7,3) = 0; Jdot(7,4) = 0;

    % Fila 8: [1, 0, 0, 0] -> constante
    % Jdot(8,:) = [0 0 0 0];

    % --- Bloque 3 (filas 9–12) con g2 ---
    % Fila 9: [ -(l1*s(a)+g2*s(b))*cth, -g2*s(b)*cth, 0, -(l1*c(a)+g2*c(b))*sth ]
    Jdot(9,1) = -(A2gdot*cth + A2g*cthdot);
    Jdot(9,2) = -(Ag2dot*cth + Ag2*cthdot);
    Jdot(9,3) = 0;
    Jdot(9,4) = -(C2gdot*sth + C2g*sthdot);

    % Fila 10: [ -(l1*s(a)+g2*s(b))*sth, -g2*s(b)*sth, 0, (l1*c(a)+g2*c(b))*cth ]
    Jdot(10,1) = -(A2gdot*sth + A2g*sthdot);
    Jdot(10,2) = -(Ag2dot*sth + Ag2*sthdot);
    Jdot(10,3) = 0;
    Jdot(10,4) =  (C2gdot*cth + C2g*cthdot);

    % Fila 11: [ l1*c(a)+g2*c(b), g2*c(b), 0, 0 ]
    Jdot(11,1) = C2gdot; Jdot(11,2) = Cg2dot; Jdot(11,3) = 0; Jdot(11,4) = 0;

    % Fila 12: [1, 1, 0, 0] -> constante
    % Jdot(12,:) = [0 0 0 0];

    % --- Bloque 4 (filas 13–16) con g3 ---
    % Fila 13: [ -(l1*s(a)+l2*s(b)+g3*s(g))*cth, -(l2*s(b)+g3*s(g))*cth, -g3*s(g)*cth, -(l1*c(a)+l2*c(b)+g3*c(g))*sth ]
    Jdot(13,1) = -(A3gdot*cth + A3g*cthdot);
    Jdot(13,2) = -( (l2*c(beta)*bdot + g3*c(gamma)*gdot)*cth + (l2*s(beta)+g3*s(gamma))*cthdot );
    Jdot(13,3) = -(Ag3dot*cth + Ag3*cthdot);
    Jdot(13,4) = -(C3gdot*sth + C3g*sthdot);

    % Fila 14: [ -(l1*s(a)+l2*s(b)+g3*s(g))*sth, -(l2*s(b)+g3*s(g))*sth, -g3*s(g)*sth, (l1*c(a)+l2*c(b)+g3*c(g))*cth ]
    Jdot(14,1) = -(A3gdot*sth + A3g*sthdot);
    Jdot(14,2) = -( (l2*c(beta)*bdot + g3*c(gamma)*gdot)*sth + (l2*s(beta)+g3*s(gamma))*sthdot );
    Jdot(14,3) = -(Ag3dot*sth + Ag3*sthdot);
    Jdot(14,4) =  (C3gdot*cth + C3g*cthdot);

    % Fila 15: [ l1*c(a)+l2*c(b)+g3*c(g), l2*c(b)+g3*c(g), g3*c(g), 0 ]
    Jdot(15,1) = C3gdot;
    Jdot(15,2) = -(l2*s(beta)*bdot + g3*s(gamma)*gdot);
    Jdot(15,3) = Cg3dot;
    Jdot(15,4) = 0;

    % Fila 16: [0, 0, 0, 1] -> constante
    Jdot(16,:) = [0 0 0 0];
end
