function PlotAreaPRO3D(L, fig)

figure(fig);
hold on;
grid on;
% axis equal
xlabel('X'); ylabel('Y'); zlabel('Z');
% view(3);

% Longitudes de los eslabones
l1 = L(1);
l2 = L(2);
l3 = L(3);

% Radio máximo y mínimo
r1 = l1 + l2 + l3;       % Alcance máximo
r2 = abs(l1 - l2 - l3);  % Alcance mínimo

% Mallado de ángulos
th = linspace(0, 2*pi, 60);   % ángulo en XY
phi = linspace(0, pi, 30);    % ángulo de elevación

[Th, Phi] = meshgrid(th, phi);

% Coordenadas para la esfera de radio r1 (límite externo)
X1 = r1 * sin(Phi) .* cos(Th);
Y1 = r1 * sin(Phi) .* sin(Th);
Z1 = r1 * cos(Phi);

% Coordenadas para la esfera de radio r2 (límite interno)
X2 = r2 * sin(Phi) .* cos(Th);
Y2 = r2 * sin(Phi) .* sin(Th);
Z2 = r2 * cos(Phi);

% Superficie externa
surf(X1, Y1, Z1, 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'FaceColor', '#39B8A0');

% Superficie interna
surf(X2, Y2, Z2, 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'FaceColor', '#39B8A0');
hold off;
end
