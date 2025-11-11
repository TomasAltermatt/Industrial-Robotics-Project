function PlotPRO(Q, L, col, fig, ll)

alpha = Q(1);
beta = Q(2);
gamma = Q(3);
theta = Q(4);

l1 = L(1);
l2 = L(2);
l3 = L(3);

a1 = alpha;
a2 = alpha + beta;
a3 = alpha + beta + gamma;

% Radial distance r and vertical displacement s (if applicable)
r = l1*cos(a1) + l2*cos(a2) + l3*cos(a3);
s = l1*sin(a1) + l2*sin(a2) + l3*sin(a3);

% Positions of joints in workspace
P0 = [0, 0, 0];
P1 = [l1*cos(a1)*cos(theta), l1*cos(a1)*sin(theta), l1*sin(a1)];
P2 = [(l1*cos(a1)+l2*cos(a2))*cos(theta), ...
    (l1*cos(a1)+l2*cos(a2))*sin(theta), ...
    l1*sin(a1)+l2*sin(a2)];
P3 = [r*cos(theta), r*sin(theta), s];  % End-effector

% Combine all points
X = [P0(1) P1(1) P2(1) P3(1)];
Y = [P0(2) P1(2) P2(2) P3(2)];
Z = [P0(3) P1(3) P2(3) P3(3)];


% Plot
figure(fig);
hold on;
grid on;
set(gca, 'Color','#4A4A4A', 'XColor','w', 'YColor','w', 'ZColor','w')
set(gcf, 'Color','#4A4A4C')

set(ll,'XData', X, 'YData', Y, 'ZData', Z, 'color', col);
xlabel('X  [cm]');
ylabel('Y  [cm]');
zlabel('Z  [cm]');
% title('3D Robot Structure', 'Color','W');
axis equal;
view(45, 30);


% --- Centered axis and margin control ---
totalLength = l1 + l2 + l3;
lim = 1.1*totalLength;
ho = l1/2;

xlim([-lim lim]);
ylim([-lim lim]);
zlim([-ho lim]);

% Draw ground plane
fill3([-lim lim lim -lim], [-lim -lim lim lim], [-ho -ho -ho -ho], [0.8 0.8 0.8], 'FaceAlpha',0.13);

end