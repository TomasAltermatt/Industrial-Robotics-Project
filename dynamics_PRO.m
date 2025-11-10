clear all; close all; clc;

%% Parameters
% robot parameters
l1 = 50e-2; % [m]
l2 = l1/1.4;
l3 = l2/1.4;
g1 = l1/2; %[m]
g2 = l2/2; %[m]
g3 = l3/2; %[m]

L=[l1;l2;l3;g1;g2;g3]; %[m]
J_rob = zeros(3, 3, 3);

% Change these
J1 = eye(3);
J2 = eye(3);
J3 = eye(3);
J_rob(:,:,1) = J1;
J_rob(:,:,2) = J2;
J_rob(:,:,3) = J3;

m_rob = [5, 5, 5];
F_rob = -9.81*m_rob;

% Payload
m_pl = 0.5;
J_pl = eye(3);
fz = -9.81*m_pl;

% Mass Matrix
M=mass_matrix(m_pl, J_pl, J_rob, m_rob);

% External forces
Fse=zeros(16,1);
Fse(3) = fz;
Fse(7) = F_rob(1);
Fse(11) = F_rob(2);
Fse(15) = F_rob(3);


%% Planning Motion curve
n_joints = 4;
Q_seq = [];
% End Effector Position (Initial & Final)

S1 = [0; 30e-2; 5e-2; -pi/6];
S8 = [0; 60e-2; 5e-2; -pi/6];
seq = Trajectory_1(0.15, 0.18, 10, S1, S8);

% S1 = [0; 30e-2; 5e-2; -pi/6];
% S2 = [0; 35e-2; 5e-2; -pi/6];
% S3 = [0; 40e-2; 5e-2; -pi/6];
% S4 = [0; 45e-2; 5e-2; -pi/6];
% S5 = [0; 50e-2; 5e-2; -pi/6];
% S6 = [0; 55e-2; 5e-2; -pi/6];
% S7 = [0; 57.5e-2; 5e-2; -pi/6];
% S8 = [0; 60e-2; 5e-2; -pi/6];
% S9 = [0; 58e-2; 7.5e-2; -pi/6];
% S10 = [0; 56e-2; 10e-2; -pi/6];
% S11 = [0; 58e-2; 12.5e-2; -pi/3];
% S12 = [0; 60e-2; 15e-2; -pi/3];
% seq = [S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11, S12];
% seq = [S1, S4, S7, S9, S11];

for i=1:length(seq(1,:))
    Q_seq(:,i) = PROinv2(seq(:,i), L, -1);
end

motor.A = [3; 3; 3; 3];
motor.D = [3; 3; 3; 3];
motor.V = [4; 4; 4; 4];

[joint_positions, joint_velocities, joint_accelerations, time_vect] = PROlines_parabolas(Q_seq, n_joints, motor);

Fq = zeros(n_joints, length(time_vect));
S_vec = zeros(4, length(time_vect));

for i = 1:length(time_vect)
    
    Q = joint_positions(:,i);
    Qp = joint_velocities(:,i);
    Qpp = joint_accelerations(:,i);
    
    % Workspace Positions
    S_vec(:,i) = PROdir(Q, L);
    
    % Extended Jacobians
    Je = PROjacdin(Q, L);
    Jep = PROjacPdin2(Q, Qp, L);
    Spp = Jep*Qp + Je*Qpp;

    Fq(:,i) = -Je'*(-M*Spp + Fse);
end



%plot forces
% figure;
% hold on
% grid on;
colors1 = lines(n_joints);
for j = 1:n_joints
    figure;
    grid on
    plot(time_vect, Fq(j,:),'Color', colors1(j,:), ...
        'DisplayName', sprintf('$q_{%d}$', j), LineWidth=1.5);
    title(sprintf('Joint %d Torques', j));
    ylabel(sprintf('$C_{%d} [N/m]$', j), Interpreter='latex')

end
xlabel('t [s]', Interpreter='latex')
hold off

% Plot Trajectory
figure;
plot(S_vec(2,:), S_vec(3,:));
hold on
xlabel('y')
ylabel('z')
% seq(1,:)
% seq(2,:)
% seq(3,:)
scatter(seq(2,:),seq(3,:),'red')

% Plot Trajectory: Reference vs Result
figure;
hold on; grid on; axis equal; view(3);

% Result trajectory (from forward kinematics)
plot3(S_vec(1,:), S_vec(2,:), S_vec(3,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Resampled trajectory');

% Reference trajectory (desired waypoints)
plot3(seq(1,:), seq(2,:), seq(3,:), 'ro--', 'LineWidth', 1.5, 'MarkerSize', 6, 'DisplayName', 'Reference trajectory');

xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title('End-Effector Trajectory');
legend('show');

