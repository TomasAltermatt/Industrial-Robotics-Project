
% Base Cyilinder
Rb = 10e-2;
Lb = 20e-2;
Ib = [1, 1, 1];

% Link 1 arm
a1 = 50e-2;
d1 = 10e-2;
w1 = 5e-2;
m1 = 5;
I1 = [1, 1, 1];

% Link 2 arm
a2 = a1/1.4;
d2 = 10e-2;
w2 = 5e-2;
m2 = 5;
I2 = [1, 1, 1];

% Link 3 arm
a3 = a2/1.4;
d3 = 10e-2;
w3 = 5e-2;
m3 = 5;
I3 = [1, 1, 1];

%% Planning Motion curve
n_joints = 4;
Q_seq = [];
% End Effector Position (Initial & Final)
S1 = [0; 30e-2; 5e-2; -pi/4];
S2 = [0; 35e-2; 5e-2; -pi/4];
S3 = [0; 40e-2; 5e-2; -pi/4];
S4 = [0; 45e-2; 5e-2; -pi/4];
S5 = [0; 50e-2; 5e-2; -pi/4];
S6 = [0; 55e-2; 5e-2; -pi/4];
S7 = [0; 60e-2; 5e-2; 0];
S8 = [0; 59e-2; 7.5e-2; 0];
S9 = [0; 58e-2; 10e-2; -pi/6];
S10 = [0; 59e-2; 12.5e-2; -pi/3];
S11 = [0; 60e-2; 15e-2; -pi/3];
% seq = [S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11];
seq = [S1, S4, S7, S9, S11];

L = [a1, a2, a3];

for i=1:length(seq(1,:))
    Q_seq(:,i) = PROinv2(seq(:,i), L, 1);
end

motor.A = [3; 3; 3; 3];
motor.D = [3; 3; 3; 3];
motor.V = [4; 4; 4; 4];

[joint_positions, joint_velocities, joint_accelerations, time_vect] = PROlines_parabolas(Q_seq, n_joints, motor);
% 1. Create the Time-Series Object for Position
q_ts_j1 = timeseries(joint_positions(1,:), time_vect(1,:), 'Name', 'Joint1Position');
q_ts_j2 = timeseries(joint_positions(2,:), time_vect(2,:), 'Name', 'Joint2Position');
q_ts_j3 = timeseries(joint_positions(3,:), time_vect(3,:), 'Name', 'Joint3Position');
q_ts_j4 = timeseries(joint_positions(4,:), time_vect(4,:), 'Name', 'Joint4Position');

% 2. Create the Time-Series Object for Velocity
q_dot_ts_j1 = timeseries(joint_velocities(1,:), time_vect(1,:), 'Name', 'Joint1Velocity');
q_dot_ts_j2 = timeseries(joint_velocities(2,:), time_vect(2,:), 'Name', 'Joint2Velocity');
q_dot_ts_j3 = timeseries(joint_velocities(3,:), time_vect(3,:), 'Name', 'Joint3Velocity');
q_dot_ts_j4 = timeseries(joint_velocities(4,:), time_vect(4,:), 'Name', 'Joint4Velocity');


% 3. Create the Time-Series Object for Acceleration
q_ddot_ts_j1 = timeseries(joint_accelerations(1,:), time_vect(1,:), 'Name', 'Joint1Acceleration');
q_ddot_ts_j2 = timeseries(joint_accelerations(2,:), time_vect(2,:), 'Name', 'Joint2Acceleration');
q_ddot_ts_j3 = timeseries(joint_accelerations(3,:), time_vect(3,:), 'Name', 'Joint3Acceleration');
q_ddot_ts_j4 = timeseries(joint_accelerations(4,:), time_vect(4,:), 'Name', 'Joint4Acceleration');

% You only need one set of time data (t) for all of them.