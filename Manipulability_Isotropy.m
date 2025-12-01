clear all
close all
clc

%% Parámetros del robot
L1 = 0.5;
L2 = L1/1.4;
L3 = L2/1.4;

L = [L1, L2, L3];

%% Rango de ángulos
q1s = linspace(deg2rad(10), deg2rad(170), 80);
q2s = linspace(deg2rad(60), deg2rad(300), 120);
q3s = linspace(deg2rad(30), deg2rad(330), 150);


%% Parámetros del mapa
res = 1e-2;  
xlim = [-1.2*sum(L), 1.2*sum(L)];
ylim = [-1.2*sum(L), 1.2*sum(L)];
xgrid = xlim(1):res:xlim(2);
ygrid = ylim(1):res:ylim(2);

manip_map = zeros(length(ygrid), length(xgrid));  % mapa de manipulabilidad
iso_map    = zeros(length(ygrid), length(xgrid)); % mapa de isotropía

%% Barrido triple
for q1 = q1s
    for q2 = q2s
        for q3 = q3s
            Q = [q1; q2; q3];
            [~, ~, ~, p3] = directKinematics3R(Q, L);
            J = jacobian_3R(Q, L);

            M = J*J.';  % matriz de manipulabilidad
            lambda = det(M);

            % Índice de manipulabilidad (Yoshikawa)
            w = sqrt(lambda);

            % Índice de isotropía
            s = svd(J);

            if isempty(s) || min(s) < 1e-12
                eta = 0;
            else
                eta = min(s) / max(s);
            end

            % Convertir posición a índice de malla
            xi = round((p3(1) - xlim(1)) / res) + 1;
            yi = round((p3(2) - ylim(1)) / res) + 1;

            if xi >= 1 && xi <= length(xgrid) && yi >= 1 && yi <= length(ygrid)
                manip_map(yi, xi) = max(manip_map(yi, xi), w);
                iso_map(yi, xi)   = max(iso_map(yi, xi), eta);
            end
        end
    end
end


%% Dibujo de los dos mapas
figure('Color','w'); 

subplot(1,2,1);
imagesc(xgrid, ygrid, manip_map); axis equal; axis tight;
xlabel('r [m]'); ylabel('z [m]');
title('Manipulability Map (Yoshikawa)');
set(gca, 'YDir', 'normal');
colormap default;
cb1 = colorbar;
cb1.Label.String = '√det(JJ^T)';

subplot(1,2,2);
imagesc(xgrid, ygrid, iso_map); axis equal; axis tight;
xlabel('r [m]'); ylabel('z [m]');
title('Isotropy Map');
set(gca, 'YDir', 'normal');
colormap default;
cb2 = colorbar;
cb2.Label.String = 'Isotropy Index (η)';

%% Normalización con curva de saturación
eta_thr = 0.7;  % umbral de isotropía aceptable
iso_map_curved = iso_map / eta_thr;
iso_map_curved(iso_map_curved > 1) = 1;

%% Dibujo del mapa saturado
figure('Color','w'); clf;
imagesc(xgrid, ygrid, iso_map_curved); axis equal; axis tight;
xlabel('r [m]'); ylabel('z [m]');
title(sprintf('Isotropy Map (Threshold η ≥ %.2f)', eta_thr));
set(gca, 'YDir', 'normal');

colormap default;
cb = colorbar;
cb.Label.String = 'Isotropy Index (η)';
cb.Ticks = [0 0.5 1];
cb.TickLabels = {sprintf('%.2g',0), sprintf('%.2g',eta_thr/2), sprintf('≥%.2g',eta_thr)};

%% ================= Funciones auxiliares =================

function [p0, p1, p2, p3] = directKinematics3R(Q, L)
q1 = Q(1); q2 = Q(2); q3 = Q(3);
p0 = [0;0];
p1 = [L(1)*cos(q1); L(1)*sin(q1)];
p2 = p1 + [L(2)*cos(q1+q2); L(2)*sin(q1+q2)];
p3 = p2 + [L(3)*cos(q1+q2+q3); L(3)*sin(q1+q2+q3)];
end

function J = jacobian_3R(Q, L)
q1 = Q(1); q2 = Q(2); q3 = Q(3);

dx_dq1 = -L(1)*sin(q1) - L(2)*sin(q1+q2) - L(3)*sin(q1+q2+q3);
dx_dq2 = -L(2)*sin(q1+q2) - L(3)*sin(q1+q2+q3);
dx_dq3 = -L(3)*sin(q1+q2+q3);
dy_dq1 =  L(1)*cos(q1) + L(2)*cos(q1+q2) + L(3)*cos(q1+q2+q3);
dy_dq2 =  L(2)*cos(q1+q2) + L(3)*cos(q1+q2+q3);
dy_dq3 =  L(3)*cos(q1+q2+q3);

J = [dx_dq1, dx_dq2, dx_dq3;
     dy_dq1, dy_dq2, dy_dq3];
end
