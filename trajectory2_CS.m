close all
clear
clc

addpath('Functions')

%% Data

n_joints = 4;
Q_seq = [];

% Length of the links
l1 = 341.000e-3; % [m]
l2 = 272.000e-3;
l3 = 181e-3;
L=[l1; l2; l3]';

% End Effector Position (Initial & Final)
S1 = [0; 30e-2; 5e-2; -pi/2];
S8 = [12.5e-2; 40e-2; 5e-2; -pi/4];
seq = Trajectory_2(0.15, 0.18, 10, S1, S8);

for i=1:length(seq(1,:))
    Q_seq(:,i) = PROinv2(seq(:,i), L(1:3), -1);
end

motor.A = [1; 1; 1; 1]*10^1;
motor.D = [1; 1; 1; 1]*5*10^0;
motor.V = [1; 1; 1; 1]*10^2;

% Adapt based on desired Workspace Positions, not arbitrarily
% Each q represents a joint, each entry represents a position
% q1 = deg2rad([0, 45, 135, 90]);
% q2 = deg2rad([0, 45, 90, 180]);
% q3 = deg2rad([0, 30, 120, 90]);
% q4 = deg2rad([0, 15, 90, 60]);
% 
% Q = [q1; q2; q3; q4];

[pos_LP, vel_LP, acc_LP, t_vect_LP] = PROcubic_splines(Q_seq, n_joints, motor);
colors1 = lines(n_joints);

% run = 1;
% while run == 1
%     i = input("Which joint do you want to plot?\n");
%     if (i <= 0) || (i > n_joints)
%         disp("Invalid joint index")
%         continue
%     end
% 
% 
%     
% 
%     % position
%     % subplot(3, n_joints, i);
%     figure;
%     grid on;
%     hold on
%     plot(t_vect_LP, pos_LP(i,:), ...
%         'DisplayName', sprintf('Lines and Parabolas'), LineWidth=1.5);
%     % plot(t_vect_CS, pos_CS(i,:), ...
%     %     'DisplayName', sprintf('Cubic Splines'), LineWidth=1.5);
%     title(sprintf('Joint %d Position', i));
%     xlabel('t [s]', Interpreter='latex')
%     ylabel('q [rad]', Interpreter='latex')
%     legend show
% 
% 
%     % velocity
%     % subplot(3, n_joints, i+4);
%     figure;
%     grid on;
%     hold on;
%     plot(t_vect_LP, vel_LP(i,:),  ...
%         'DisplayName', sprintf('Lines and Parabolas'), LineWidth=1.5);
%     % plot(t_vect_CS, vel_CS(i,:), ...
%     %     'DisplayName', sprintf('Cubic Splines'), LineWidth=1.5);
%     title(sprintf('Joint %d Velocity', i));
%     xlabel('t [s]', Interpreter='latex')
%     ylabel('$\dot q$ [rad/s]', Interpreter='latex')
%     legend show
% 
% 
%     % acceleration
%     % subplot(3, n_joints, i+8);
%     figure;
%     grid on;
%     hold on;
%     plot(t_vect_LP, acc_LP(i,:), ...
%         'DisplayName', sprintf('Lines and Parabolas'), LineWidth=1.5);
%     % plot(t_vect_CS, acc_CS(i,:), ...
%     %     'DisplayName', sprintf('Cubic Splines'), LineWidth=1.5);
%     title(sprintf('Joint %d Acceleration', i));
%     xlabel('t [s]', Interpreter='latex')
%     ylabel('$\ddot q$ [rad/$s^2$]', Interpreter='latex')
%     legend show
% 
%     cont = input("Plot other Joints?\n [1] Yes\n [0] No\n");
%     if cont == 1
%         close all
%     else
%         break
%     end
% 
% end


for j = 1:n_joints
    figure(1);
    hold on
    grid on;
    plot(t_vect_LP, pos_LP(j,:),'Color', colors1(j,:), ...
        'DisplayName', sprintf('$q_{%d}$', j), LineWidth=1.5);
    title('Joint Positions (ADE Trajectory)');
    xlabel('t [s]', Interpreter='latex')
    ylabel('$q [rad]$', Interpreter='latex')
    L = legend('show', 'Location','best');
    set(L, 'Interpreter', 'latex')


    figure(2);
    hold on
    grid on;
    plot(t_vect_LP, vel_LP(j,:),'Color', colors1(j,:), ...
        'DisplayName', sprintf('$q_{%d}$', j), LineWidth=1.5);
    title('Joint Velocities (ADE Trajectory)');
    xlabel('t [s]', Interpreter='latex')
    ylabel('$\dot q [rad/s]$', Interpreter='latex')
    L = legend('show', 'Location','best');
    set(L, 'Interpreter', 'latex')


    figure(3);
    hold on
    grid on;
    plot(t_vect_LP, acc_LP(j,:),'Color', colors1(j,:), ...
        'DisplayName', sprintf('$q_{%d}$', j), LineWidth=1.5);
    title('Joint Accelerations (ADE Trajectory)');
    xlabel('t [s]', Interpreter='latex')
    ylabel('$\ddot q [rad/s^2]$', Interpreter='latex')
    L = legend('show', 'Location','best');
    set(L, 'Interpreter', 'latex')

end








