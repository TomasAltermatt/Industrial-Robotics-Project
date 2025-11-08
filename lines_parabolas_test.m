close all
clear
clc

%% Data

n_joints = 4;
Q = [];


% End Effector Position (Initial & Final)
S1 = [0; 30e-2; 5e-2; -pi/6];
S2 = [0; 35e-2; 5e-2; -pi/6];
S3 = [0; 40e-2; 5e-2; -pi/6];
S4 = [0; 45e-2; 5e-2; -pi/6];
S5 = [0; 50e-2; 5e-2; -pi/6];
S6 = [0; 55e-2; 5e-2; -pi/6];
S7 = [0; 57.5e-2; 5e-2; -pi/6];
S8 = [0; 60e-2; 5e-2; -pi/6];
S9 = [0; 58e-2; 7.5e-2; -pi/6];
S10 = [0; 56e-2; 10e-2; -pi/6];
S11 = [0; 58e-2; 12.5e-2; -pi/3];
S12 = [0; 60e-2; 15e-2; -pi/3];
seq = [S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11, S12];


% Length of the links
l1 = 50e-2; % [m]
l2 = l1/1.4;
l3 = l2/1.4;
L=[l1; l2; l3]';

for i=1:length(seq(1,:))
    Q(:,i) = PROinv2(seq(:,i), L, 1);
end

motor.A = [3; 3; 3; 3];
motor.D = [3; 3; 3; 3];
motor.V = [4; 4; 4; 4];

% Adapt based on desired Workspace Positions, not arbitrarily
% Each q represents a joint, each entry represents a position
% q1 = deg2rad([0, 45, 135, 90]);
% q2 = deg2rad([0, 45, 90, 180]);
% q3 = deg2rad([0, 30, 120, 90]);
% q4 = deg2rad([0, 15, 90, 60]);
% 
% Q = [q1; q2; q3; q4];

[joint_positions, joint_velocities, joint_accelerations, time_vect] = PROlines_parabolas(Q, n_joints, motor);


run = 1;
while run == 1
    i = input("Which joint do you want to plot?\n");
    if (i <= 0) || (i > n_joints)
        disp("Invalid joint index")
        continue
    end

    figure;
    grid on;
    colors1 = lines(n_joints);
    
    % position
    % subplot(3, n_joints, i);
    plot(time_vect, joint_positions(i,:),'Color', colors1(i,:), ...
        'DisplayName', sprintf('$q_{%d}$', i), LineWidth=1.5);
    title(sprintf('Joint %d Position', i));
    xlabel('t [s]', Interpreter='latex')
    ylabel('q [rad]', Interpreter='latex')
    
    figure;
    grid on;
    % velocity
    % subplot(3, n_joints, i+4);
    plot(time_vect, joint_velocities(i,:),'Color', colors1(i,:), ...
        'DisplayName', sprintf('$q_{%d}$', i), LineWidth=1.5);
    title(sprintf('Joint %d Velocity', i));
    xlabel('t [s]', Interpreter='latex')
    ylabel('$\dot q$ [rad/s]', Interpreter='latex')
    
    figure;
    grid on;
    % acceleration
    % subplot(3, n_joints, i+8);
    plot(time_vect, joint_accelerations(i,:),'Color', colors1(i,:), ...
        'DisplayName', sprintf('$q_{%d}$', i), LineWidth=1.5);
    title(sprintf('Joint %d Acceleration', i));
    xlabel('t [s]', Interpreter='latex')
    ylabel('$\ddot q$ [rad/$s^2$]', Interpreter='latex')
    
    cont = input("Plot other Joints?\n [1] Yes\n [0] No\n");
    if cont == 1
        close all
    else
        break
    end

end

show_all = input("Show all plots?\n [1] Yes\n [0] No\n");
if show_all == 1
    close all
    for j = 1:n_joints
        figure(1);
        hold on
        grid on;
        subplot(n_joints, 1, j)
        plot(time_vect, joint_positions(j,:),'Color', colors1(j,:), ...
            'DisplayName', sprintf('$q_{%d}$', j), LineWidth=1.5);
        title(sprintf('Joint %d Position', j));
        xlabel('t [s]', Interpreter='latex')
        ylabel('$q [rad]$', Interpreter='latex')

        figure(2);
        hold on
        grid on;
        subplot(n_joints, 1, j)
        plot(time_vect, joint_velocities(j,:),'Color', colors1(j,:), ...
            'DisplayName', sprintf('$q_{%d}$', j), LineWidth=1.5);
        title(sprintf('Joint %d Velocity', j));
        xlabel('t [s]', Interpreter='latex')
        ylabel('$\dot q [rad/s]$', Interpreter='latex')


        figure(3);
        hold on
        grid on;
        subplot(n_joints, 1, j)
        plot(time_vect, joint_accelerations(j,:),'Color', colors1(j,:), ...
            'DisplayName', sprintf('$q_{%d}$', j), LineWidth=1.5);
        title(sprintf('Joint %d Acceleration', j));
        xlabel('t [s]', Interpreter='latex')
        ylabel('$\ddot q [rad/s^2]$', Interpreter='latex')

    end
end






