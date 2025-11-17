clear all; close all; clc;
addpath('Functions')

%% Data
% Base Parameters
Rb = 10e-2;
Lb = 20e-2;
Ib = [1, 1, 1];  % Inertia Moment
Ibp = [0, 0, 0]; % Inertia Product
Ib_tot = inertia_matrix(Ib, Ibp);

% Link 1 arm Parameters
a1 = 50e-2;         % length
d1 = 5e-2;          % height
w1 = 5e-2;          % width
m1 = 5;             % link mass
g1 = 0.5* a1;       % center of mass
I1 = [ 3.8620847e+03,1.7221182e+04,1.9444675e+04];     % Inertia Moment
I1p = [ -3.2999403e+03 ,  3.5137952e+01 ,  3.2038278e+01];    % Inertia Product
% Calculate the inertia matrix for Link 1
I1_tot = inertia_matrix(I1, I1p);

% Link 2 arm Parameters
a2 = a1/1.4;        % length
d2 = 4e-2;          % height
w2 = 4e-2;          % width
m2 = 5;             % link mass
g2 = 0.5*a2;        % center of mass
I2 = [3.8666085e+03, 1.7241799e+04,   1.9469790e+04];     % Inertia Moment
I2p = [-3.3096372e+03, 3.5119295e+01, 3.2029042e+01];    % Inertia Product
% Calculate the inertia matrix for Link 2
I2_tot = inertia_matrix(I2, I2p);

% Link 3 arm Parameters
a3 = a2/1.4;        % length
d3 = 3e-2;          % height
w3 = 3e-2;          % width
m3 = 5;             % link mass
g3 = 0.5*a3;        % center of mass
I3 = [5.7807216e+02, 2.8034581e+03, 2.3138400e+03];     % Inertia Moment
I3p = [0, 0, 0];    % Inertia Product
% Calculate the inertia matrix for Link 3
I3_tot = inertia_matrix(I3, I3p);


% Save 3 inertia tensors
J_rob = zeros(3, 3, 4);
J_rob(:,:,1) = I1_tot;
J_rob(:,:,2) = I2_tot;
J_rob(:,:,3) = I3_tot;
J_rob(:,:,4) = Ib_tot;


% Robot mass/forces
m_rob = [m1, m2, m3];
F_rob = -9.81*m_rob;

% Payload
m_pl = 0.5;
Ipl = [1, 1, 1];
Iplp = [0, 0, 0];
J_pl = inertia_matrix(Ipl, Iplp);
fz = -9.81*m_pl;

% Mass Matrix
M=mass_matrix(m_pl, J_pl, J_rob, m_rob);

% External forces
Fse=zeros(16,1);
Fse(3) = fz;
Fse(7) = F_rob(1);
Fse(11) = F_rob(2);
Fse(15) = F_rob(3);


% Dimension vectors
L=[a1;a2;a3;g1;g2;g3]; %[m]


%% Planning Motion curve
n_joints = 4;
Q_seq = [];
% End Effector Position (Initial & Final)

S1 = [0; 30e-2; 5e-2; -pi/2];
S8 = [0; 60e-2; 5e-2; -pi/4];
seq = Trajectory_1(0.15, 0.18, 10, S1, S8);

for i=1:length(seq(1,:))
    Q_seq(:,i) = PROinv2(seq(:,i), L(1:3), -1);
end

motor.A = [3; 3; 3; 3];
motor.D = [3; 3; 3; 3];
motor.V = [4; 4; 4; 4];

[joint_positions, joint_velocities, joint_accelerations, time_vect] = PROcubic_splines(Q_seq, n_joints, motor);
% time_vect = 0:0.1:5;
% time_vect = time_vect;
% joint_positions = zeros(n_joints, length(time_vect));
% joint_velocities = zeros(n_joints, length(time_vect));
% joint_accelerations = zeros(n_joints, length(time_vect));
% 
% for i = 1:n_joints
%     joint_positions(i,:) = sin(time_vect);
%     joint_velocities(i,:) = cos(time_vect);
%     joint_accelerations(i,:) = -sin(time_vect);
% end


%% Analytical Model
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

%% Simscape Model

% 1. Create the Time-Series Object for Position
q_ts_j1 = timeseries(joint_positions(1,:), time_vect, 'Name', 'Joint1Position');
q_ts_j2 = timeseries(joint_positions(2,:), time_vect, 'Name', 'Joint2Position');
q_ts_j3 = timeseries(joint_positions(3,:), time_vect, 'Name', 'Joint3Position');
q_ts_j4 = timeseries(joint_positions(4,:), time_vect, 'Name', 'Joint4Position');

% 2. Create the Time-Series Object for Velocity
q_dot_ts_j1 = timeseries(joint_velocities(1,:), time_vect, 'Name', 'Joint1Velocity');
q_dot_ts_j2 = timeseries(joint_velocities(2,:), time_vect, 'Name', 'Joint2Velocity');
q_dot_ts_j3 = timeseries(joint_velocities(3,:), time_vect, 'Name', 'Joint3Velocity');
q_dot_ts_j4 = timeseries(joint_velocities(4,:), time_vect, 'Name', 'Joint4Velocity');


% 3. Create the Time-Series Object for Acceleration
q_ddot_ts_j1 = timeseries(joint_accelerations(1,:), time_vect, 'Name', 'Joint1Acceleration');
q_ddot_ts_j2 = timeseries(joint_accelerations(2,:), time_vect, 'Name', 'Joint2Acceleration');
q_ddot_ts_j3 = timeseries(joint_accelerations(3,:), time_vect, 'Name', 'Joint3Acceleration');
q_ddot_ts_j4 = timeseries(joint_accelerations(4,:), time_vect, 'Name', 'Joint4Acceleration');


out = sim('DOF4_v1.slx');

C1 = out.J1_torque;
C2 = out.J2_torque;
C3 = out.J3_torque;
C4 = out.J4_torque;
Fq_ss = [C1, C2, C3, C4]';
time_ss = out.time_ss;

%plot forces
% figure;
% hold on
% grid on;
colors1 = lines(n_joints);
for j = 1:n_joints
    figure;
    plot(time_vect, Fq(j,:),'DisplayName', 'Analytical', LineWidth=1.5);
    hold on;
    grid on;
    plot(time_ss, Fq_ss(j,:), 'DisplayName', 'Simscape', LineWidth=1.5);
    title(sprintf('Joint %d Torques', j));
    ylabel(sprintf('$C_{%d} [N/m]$', j), Interpreter='latex')
    legend('show')
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

