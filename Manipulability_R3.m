clear all
% close all
clc

%% Parámetros del robot
L1 = 0.5;
L2 = L1/1.4;
L3 = L2/1.4;

L = [L1, L2, L3];

%% Rango de ángulos
q1s = linspace(deg2rad(0), deg2rad(180), 50);
q2s = linspace(deg2rad(60), deg2rad(300), 60);
q3s = linspace(deg2rad(30), deg2rad(330), 70);


%% Parametros del mapa
res = 1e-2;  % resolución espacial (1 cm)
xlim = [-1.2, 1.2];  % límites del plano
ylim = [-1.2, 1.2];

xgrid = xlim(1):res:xlim(2);
ygrid = ylim(1):res:ylim(2);
sigma_map = zeros(length(ygrid), length(xgrid));  % mapa de manipulabilidad

%% Barrido triple

for q1 = q1s
    for q2 = q2s
        for q3 = q3s
            Q = [q1; q2; q3];

            [~, ~, ~, p3] = directKinematics3R(Q, L);
            J = jacobian_3R(Q, L);

            M = J*J.';

            lambda = det(M);
            sigma = sqrt(lambda);  % índice de manipulabilidad

            % Converte posicion a índice de malla
            xi = round((p3(1) - xlim(1)) / res) + 1;
            yi = round((p3(2) - ylim(1)) / res) + 1;

            if xi >= 1 && xi <= length(xgrid) && yi >= 1 && yi <= length(ygrid)
                sigma_map(yi, xi) = max(sigma_map(yi, xi), sigma);  % guardar máximo
            end
        end
    end
end


%% Normalización
sigma_min = min(sigma_map(:));
sigma_max = max(sigma_map(:));


%% Dibujo del mapa
figure('Color','w'); clf;
imagesc(xgrid, ygrid, sigma_map); axis equal; axis tight;
xlabel('r [m]'); ylabel('z [m]');
title('Manipulability Map');
set(gca, 'YDir', 'normal');  % para que el eje Y no esté invertido

colormap default;
cb = colorbar;
cb.Label.String = 'Manipulability Index Value';
cb.Ticks = linspace(0, sigma_max, 5);
cb.TickLabels = arrayfun(@(x) sprintf('%.2g', x), cb.Ticks, 'UniformOutput', false);


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
