clear all; close all; clc;
addpath('Functions')

%% Data
% Base Parameters
Rb = 10e-2;
Lb = 20e-2;
m_base = 1.2271304; % [kg]
g_base_vector = [1.30E+02	4.68E+01	4.42E-01]*10^-3;
g_base = norm(g_base_vector);
Ib_tot = [3.26E+03	-1.62E+03	-1.36E+01;
        -1.62E+03	5.23E+03	-2.68E+01;
        -1.36E+01	-2.68E+01	6.95E+03]*10^-6;
Ib_mom = [Ib_tot(1,1), Ib_tot(2,2), Ib_tot(3,3)];
Ib_prod = [Ib_tot(1,2), Ib_tot(1,3), Ib_tot(2,3)];

% Link 1 arm Parameters
a1 = 360e-3;             % length
d1 = 4e-2;              % height
w1 = 53.823e-3;              % width
m1 = 1.9886957e+00;       % link mass
g1_vector = [2.8305972e+02  1.9847175e+01 -3.6990314e-01]*10^-3;
g1_ss = g1_vector - ([a1, 0, 0])/2;

I1_tot = [4.10E+03	-4.15E+03	4.31E+01
        -4.15E+03	2.36E+04	5.15E+01
        4.31E+01	5.15E+01	2.64E+04]*10^-6;
I1_mom = [I1_tot(1,1), I1_tot(2,2), I1_tot(3,3)];
I1_prod = [I1_tot(1,2), I1_tot(1,3), I1_tot(2,3)];


% Link 2 arm Parameters
a2 = 300e-3;        % length
d2 = 4e-2;          % height
w2 = 53.823e-3;          % width
m2 = 1.6741142e+00;             % link mass
g2_vector = [242.58	-21.483	0.383]*10^-3;
g2_ss = g2_vector - ([a2, 0, 0])/2;

I2_tot = [2.65E+03	2.12E+03	-9.67E+00
        2.12E+03	1.36E+04	1.38E+00
        -9.67E+00	1.38E+00	1.51E+04]*10^-6;
I2_mom = [I2_tot(1,1), I2_tot(2,2), I2_tot(3,3)];
I2_prod = [I2_tot(1,2), I2_tot(1,3), I2_tot(2,3)];



% Link 3 arm Parameters
a3 = 190e-3;        % length
d3 = 4e-2;          % height
w3 = 53.823e-3;          % width
m3 = 0.215;             % link mass
g3_vector = [82.123	-6.987	-0.005]*10^-3;
g3_ss = g3_vector - ([a3, 0, 0])/2;

% Calculate the inertia matrix for Link 3
I3_tot = [88.25E+01	-9.81E+01	4.07E-02
-9.81E+01	9.39E+02	-4.74E-03
4.07E-02	-4.74E-03	9.04E+02
]*10^-6;
I3_mom = [I3_tot(1,1), I3_tot(2,2), I3_tot(3,3)];
I3_prod = [I3_tot(1,2), I3_tot(1,3), I3_tot(2,3)];


% Save 3 inertia tensors
J_rob = zeros(3, 3, 4);
J_rob(:,:,1) = I1_tot;
J_rob(:,:,2) = I2_tot;
J_rob(:,:,3) = I3_tot;
J_rob(:,:,4) = Ib_tot;


% Robot mass/forces
m_rob = [m1, m2, m3];
F_rob = -9.81*m_rob;

% Linear Spring Parameters
k = 5e3;     % Linear Spring stiffness (N/m)
x0 = 16e-2;  % Spring preload (m)
Cc = 128.47e-3; % Center to center distance (m)
r = 51.4e-3;   % Arm distance (m)

% Payload
m_pl = 1;
Ipl = [0, 0, 0];
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
L = [a1; a2; a3; ...
     g1_vector(1); g1_vector(2); g1_vector(3); ... 
     g2_vector(1); g2_vector(2)-w1; g2_vector(3); ...
     g3_vector(1); g3_vector(2)+w2; g3_vector(3)];


%% Planning Motion curve
n_joints = 4;
Q_seq = [];
% End Effector Position (Initial & Final)

S1 = [0; 30e-2; 5e-2; -pi/2];
S8 = [0; 40e-2; 5e-2; -pi/4];
seq = Trajectory_1(0.15, 0.18, 10, S1, S8);

for i=1:length(seq(1,:))
    Q_seq(:,i) = PROinv2(seq(:,i), L(1:3), -1);
end

% ASK HOW WE CHANGE THESE VALUES CONSIDERING TRANSMISSION SYSTEM AND WHERE
% DO WE FIND THESE NOMINAL VALUES
motor.A = [1; 1; 1; 1]*10^1;
motor.D = [1; 1; 1; 1]*5*10^0;
motor.V = [1; 1; 1; 1]*10^2;

[joint_positions, joint_velocities, joint_accelerations, time_vect] = PROlines_parabolas(Q_seq, n_joints, motor);


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
    Je = PROjacdinV2(Q, L);
    Jep = PROjacPdinV2(Q, Qp, L);
    Spp = Jep*Qp + Je*Qpp;
    
    % Spring Contribution
    Ls = sqrt( (Cc - r*sin(Q(1)) )^2 +  (r*cos(Q(1)))^2);
    Fs = -k * (Ls - x0);
    F_spring = Fs*r*Cc*cos(Q(1))/sqrt(r^2 + Cc^2 - 2*r*Cc*sin(Q(1)));
    Fse(8)= F_spring; % Add spring force contribution to the first joint

    Fq(:,i) = -Je'*(-M*Spp + Fse);
end

%% Simscape Model

% Torque time vector
Q = joint_positions(1,:);
% Spring Contribution
Ls = sqrt( (Cc - r*sin(Q)).^2 +  (r*cos(Q)).^2);
Fs_vect = -k * (Ls - x0);
F_spring_vect = Fs_vect*r*Cc.*cos(Q)./sqrt(r^2 + Cc^2 - 2*r*Cc*sin(Q));

% Create timeseries for External torque vector
F_ts_spring = timeseries(F_spring_vect, time_vect, 'Name', 'Joint1SpringTorque');
F_ts_spring_neg = timeseries(-F_spring_vect, time_vect, 'Name', 'Joint1SpringTorque');

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
    plot(time_vect, Fq(j,:),'DisplayName', 'Analytical', LineWidth=2);
    hold on;
    grid on;
    plot(time_ss, Fq_ss(j,:), 'DisplayName', 'Simscape','LineStyle','-','Color','r', LineWidth=1);
    title(sprintf('Joint %d Torques', j));
    ylabel(sprintf('$C_{%d} [N*m]$', j), Interpreter='latex')
    legend('show')
    
    % figure;
    % torque_diff = Fq_ss(j,:) - Fq(j,:);
    % plot(time_ss, torque_diff,'LineStyle','-','Color','r', LineWidth=1);
    % grid on
    % title(sprintf('Joint %d Torques Difference', j));
    % ylabel(sprintf('$C_{%d} [N*m]$', j), Interpreter='latex')

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

