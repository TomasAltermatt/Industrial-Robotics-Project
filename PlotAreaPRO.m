function PlotAreaPRO (L, fig)

figure(fig);
hold on;
grid on;

% Lengths of limbs
l1 = L(1);
l2 = L(2);
l3 = L(3);

% Calculate the external and internal boundary
r1 = l1 + l2 + l3;       % Maximum limit of the Robot
r2 = abs(l1 - l2 - l3);  % Minimum limit of the Robot

% The circle is based on multiple lines so we need to create the vector th
% that contains the angles for which every line is going to draw with
% respect to the previous one.
th = 0 : pi/50 : 2*pi;

% Here are the different coordinates for each line
x_internal = r2 * cos(th);
y_internal = r2 * sin(th);
z_internal = r2 * sin(th);

x_external = r1 * cos(th);
y_external = r1 * sin(th);
z_external = r1 * sin(th);

% Plot the circles
% plot(x_internal, y_internal, 'LineWidth', 1.3, 'color', '#39B8A0');
plot(x_external, y_external, 'LineWidth', 1.3, 'color', '#39B8A0');

% Dibujo en el plano XZ usando plot3
% plot3(x_internal, zeros(size(z_internal)), z_internal, 'LineWidth', 1.3, 'color', '#39B8A0');
plot3(x_external, zeros(size(z_external)), z_external, 'LineWidth', 1.3, 'color', '#39B8A0');
hold off
