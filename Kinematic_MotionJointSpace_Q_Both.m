clear
close all
clc
addpath('Functions')
%% Kinematics & Motion Profiles in Q - Joint Space


% Length of the links
% I remember the professor said something about a 1.4 relation between the
% limbs of the robot, but I know is not a golden rule.
l1 = 70;
l2 = l1/1.4;
l3 = l2/1.4;

L=[l1; l2; l3]';

% End Effector Position (Initial & Final)
Si = [-20; -65; 85; -pi/6];
Sf = [85; 33; 43; -pi/3];

% Loop that goes from 1 to -1, so it shows both configurations of the Robot.
for config = 1 : -2 : -1

    % Calculate the intial value of Q. This allows me to determine the starting
    % point of alpha an beta, considering the end effector in the position Si.
    % And I can do the same for the final position. And then calculate the
    % difference between them. (We want to plan the trajectory in Q, thats why
    % we need Q initial and final.
    Qi = PROinv2(Si, L, config);   % Initial positions
    Qf = PROinv2(Sf, L, config);   % Final positions
    dQ = Qf - Qi;               % Difference of positions in Q

    % Now I have to split it in different motion curves, in the Q space:
    q1_i = Qi(1);       % Initial position in q1 (q1_o). Takes the first element.
    dq1 = dQ(1);        % Difference between initial and final position of q1.
    
    q2_i = Qi(2);       % Initial position in q2 (q2_o). Takes the Second element.
    dq2 = dQ(2);        % Difference between initial and final position of q2.
    
    q3_i = Qi(3);       % Initial position in q3 (q3_o). Takes the first element.
    dq3 = dQ(3);        % Difference between initial and final position of q3.
    
    q4_i = Qi(4);       % Initial position in q4 (q4_o). Takes the Second element.
    dq4 = dQ(4);        % Difference between initial and final position of q4.

    % Defining the time profile for all the joints.
    T = 9;              % Total of 9 seconds
    
    % Time instants in q1 and q2 movements. Usually are the same, but can change.
    t1_q1 = 3;
    t2_q1 = 6;
    t3_q1 = T;
    
    t1_q2 = 3;
    t2_q2 = 6;
    t3_q2 = T;

    t1_q3 = 3;
    t2_q3 = 6;
    t3_q3 = T;
    
    t1_q4 = 3;
    t2_q4 = 6;
    t3_q4 = T;

    % Create the figure
    f1 = figure('color','white');
    f1.WindowStyle = 'normal';
    % f1.WindowState = 'maximized';
    scr = get(0,'ScreenSize');
    
    % Dividing plots by solutions, and to select the color of the plot
    % depending on the chosen configuration.
    if config == 1
        % Half-screen figure (bottom-left corner)
        f1.Position = [scr(1) scr(2) scr(3)/2 scr(4)];
        title('3D Robot Structure - 1^{st} Configuration', 'Color','W');
        line_color = '#C7143E';
        marker_edge_color = '#941C38';
    else
        f1.Position = [scr(3)/2 scr(2) scr(3)/2 scr(4)];
        title('3D Robot Structure - 2^{nd} Configuration', 'Color','W');
        line_color = '#422787';
        marker_edge_color = '#362563';
    end
    
    % Define the line type and properties (These are used to draw the two
    % solutions.
    ll1 = line('XData', [0 0 0], ...
               'YData', [0 0 0], ...
               'ZData', [0 0 0], ...
               'LineStyle', '-', ...
               'LineWidth', 4.2, ...
               'Color', 'k', ...
               'Marker', 'o', ...
               'MarkerSize', 5.6, ...
               'MarkerFaceColor', '#C79114', ...
               'MarkerEdgeColor', marker_edge_color);
    
    % This is just a black line in the origin with a negative length to
    % represent an offset depending of the initial heigh of the robot that we
    % want to put. We can modify later during the design.
    ll3 = line('XData', [0 0 0], 'YData', [0 0 0], 'ZData', [0 0 -l1/2], ...
        'linestyle', '-','linewidth',4.2,'color','#383436',...
        'marker','o','markersize',8.2,'markerfacecolor','#C79114');
    % This is just to see the point we want to reach
    Goal_Point = line('XData', Sf(1),'YData', Sf(2), 'ZData', Sf(3), ...
        'linestyle', '-','linewidth',1.2,'color','#27F55B',...
        'marker','o','markersize',4,'markerfacecolor','#27F55B');
    
%     % Crear objeto de video
%     filename = sprintf('Test_01 %.2f', config)
%     v = VideoWriter(filename, 'MPEG-4');    v.FrameRate = 24;
%     open(v);
    
    % Calls the function to plot the Area:
    PlotAreaPRO(L, f1); % draw the work area
    PlotAreaPRO3D(L, f1); % draw the work Volume
    pause(0.1)
        
    % Iteration Variable
    i = 1;
    dT = 0.13;

    for t = 0 : dT : T

        % Call the function one time, and receive a set of results in position,
        % velocity and acceleration, according to the profile setted before. In
        % this case one profile for each joint (Q Space)
        res_q1 = Sshape(t, q1_i, dq1, t1_q1, t2_q1, t3_q1);
        res_q2 = Sshape(t, q2_i, dq2, t1_q2, t2_q2, t3_q2);
        res_q3 = Sshape(t, q3_i, dq3, t1_q3, t2_q3, t3_q3);
        res_q4 = Sshape(t, q4_i, dq4, t1_q4, t2_q4, t3_q4); 

        % We get the results in the Q space (Joint Space)
        Q = [res_q1.pos; res_q2.pos; res_q3.pos; res_q4.pos];     % Position
        Qp = [res_q1.vel; res_q2.vel; res_q3.vel; res_q4.vel];    % Velocity
        Qpp = [res_q1.acc; res_q2.acc; res_q3.acc; res_q4.acc];   % Acceleration
    
    
        % Solution of the position required (Direct Kinematics)
        S = PROdir(Q, L)
    
        % Draw the path that the robot is performing.
        Path = line('XData', S(1),'YData', S(2), 'ZData', S(3), ...
            'linestyle', '-','linewidth',1,'color','#F59127',...
            'marker','o','markersize',2,'markerfacecolor','#C79114');
    
        % Plot the Robot in the chosen configuration
        PlotPRO(Q, L, line_color, f1, ll1); 
    
    
        % Calls the Jacobian function
        J = PROjac(Q, L);
    
        % Calculate the Velocity of the Workspace (Sp).
        Sp = J*Qp;
    
        % Calls the derivative of the Jacobian
        Jp = PROjacP(Q, Qp, L);     % In this case it uses only the first solution.
        
        % Calculate the Acceleration of the Workspace (Spp).
        Spp = Jp*Qp + J*Qpp;
    
        % Save the time instant in a vector of time.
        time(i) = t;
    
        % Save the values of position, velocity and acceleration in vectors.
        % For x axis:
        px(i) = S(1);
        vx(i) = Sp(1);
        ax(i) = Spp(1);
        % For y axis:
        py(i) = S(2);
        vy(i) = Sp(2);
        ay(i) = Spp(2);
        % For z axis:
        pz(i) = S(3);
        vz(i) = Sp(3);
        az(i) = Spp(3);
        % For phi angle:
        pfi(i) = S(4);
        vfi(i) = Sp(4);
        afi(i) = Spp(4);
        
        % Save the values of the Jointspace in vectors.
        q1(i) = Q(1);       % Angular Position in Joint 1
        q1p(i) = Qp(1);     % Angular Velocity in Joint 1
        q1pp(i) = Qpp(1);   % Angular Acceleration in Joint 1
        
        q2(i) = Q(2);       % Angular Position in Joint 2
        q2p(i) = Qp(2);     % Angular Velcocity in Joint 2
        q2pp(i) = Qpp(2);   % Angular Acceleration in Joint 2
        
        q3(i) = Q(3);       % Angular Position in Joint 3
        q3p(i) = Qp(3);     % Angular Velcocity in Joint 3
        q3pp(i) = Qpp(3);   % Angular Acceleration in Joint 3
        
        q4(i) = Q(4);       % Angular Position in Joint base
        q4p(i) = Qp(4);     % Angular Velcocity in Joint base
        q4pp(i) = Qpp(4);   % Angular Acceleration in Joint base
        
        % Increments the iteration value
        i = i + 1;
    
%         % Capturar frame
%         frame = getframe(gcf);
%         writeVideo(v, frame);
    
    end
    
%     % Cerrar el archivo de video
%     close(v);



    
    %% Plotting the profiles in Q - Joint Space
    if config == 1
        figure('Name','Joint Space Profiles - 1st Configuration','NumberTitle','off', ...
               'Position',[scr(1), scr(4)/2, scr(3)/2, scr(4)/2]);

        % Global title
        sgtitle('Q - Joint Space Profiles - 1^{st} Configuration','FontSize',14,'FontWeight','bold');
    
    else
        figure('Name','Joint Space Profiles -  2nd Configuration','NumberTitle','off', ...
               'Position',[scr(3)/2, scr(4)/2, scr(3)/2, scr(4)/2]);

        % Global title
        sgtitle('Q - Joint Space Profiles - 2^{nd} Configuration','FontSize',14,'FontWeight','bold');
    end

    axis tight;
    
    % Define colors for each joint
    colors = lines(4); % 4 distinct colors
    
    % --- Joint Positions ---
    subplot(3,4,1); plot(time, q1, 'LineWidth',1.5,'Color',colors(1,:)); grid on; axis tight;
    xlabel('Time [s]'); ylabel('q_1 [rad]'); title('Joint 1 Position'); set(gca,'FontSize',10);
    
    subplot(3,4,2); plot(time, q2, 'LineWidth',1.5,'Color',colors(2,:)); grid on; axis tight;
    xlabel('Time [s]'); ylabel('q_2 [rad]'); title('Joint 2 Position'); set(gca,'FontSize',10);
    
    subplot(3,4,3); plot(time, q3, 'LineWidth',1.5,'Color',colors(3,:)); grid on; axis tight;
    xlabel('Time [s]'); ylabel('q_3 [rad]'); title('Joint 3 Position'); set(gca,'FontSize',10);
    
    subplot(3,4,4); plot(time, q4, 'LineWidth',1.5,'Color',colors(4,:)); grid on; axis tight;
    xlabel('Time [s]'); ylabel('q_4 [rad]'); title('Joint 4 Position'); set(gca,'FontSize',10);
    
    % --- Joint Velocities ---
    subplot(3,4,5); plot(time, q1p, 'LineWidth',1.5,'Color',colors(1,:)); grid on; axis tight;
    xlabel('Time [s]'); ylabel('dq_1/dt [rad/s]'); title('Joint 1 Velocity'); set(gca,'FontSize',10);
    
    subplot(3,4,6); plot(time, q2p, 'LineWidth',1.5,'Color',colors(2,:)); grid on; axis tight;
    xlabel('Time [s]'); ylabel('dq_2/dt [rad/s]'); title('Joint 2 Velocity'); set(gca,'FontSize',10);
    
    subplot(3,4,7); plot(time, q3p, 'LineWidth',1.5,'Color',colors(3,:)); grid on; axis tight;
    xlabel('Time [s]'); ylabel('dq_3/dt [rad/s]'); title('Joint 3 Velocity'); set(gca,'FontSize',10);
    
    subplot(3,4,8); plot(time, q4p, 'LineWidth',1.5,'Color',colors(4,:)); grid on; axis tight;
    xlabel('Time [s]'); ylabel('dq_4/dt [rad/s]'); title('Joint 4 Velocity'); set(gca,'FontSize',10);
    
    % --- Joint Accelerations ---
    subplot(3,4,9); plot(time, q1pp, 'LineWidth',1.5,'Color',colors(1,:)); grid on; axis tight;
    xlabel('Time [s]'); ylabel('d^2q_1/dt^2 [rad/s^2]'); title('Joint 1 Acceleration'); set(gca,'FontSize',10);
    
    subplot(3,4,10); plot(time, q2pp, 'LineWidth',1.5,'Color',colors(2,:)); grid on; axis tight;
    xlabel('Time [s]'); ylabel('d^2q_2/dt^2 [rad/s^2]'); title('Joint 2 Acceleration'); set(gca,'FontSize',10);
    
    subplot(3,4,11); plot(time, q3pp, 'LineWidth',1.5,'Color',colors(3,:)); grid on; axis tight;
    xlabel('Time [s]'); ylabel('d^2q_3/dt^2 [rad/s^2]'); title('Joint 3 Acceleration'); set(gca,'FontSize',10);
    
    subplot(3,4,12); plot(time, q4pp, 'LineWidth',1.5,'Color',colors(4,:)); grid on; axis tight;
    xlabel('Time [s]'); ylabel('d^2q_4/dt^2 [rad/s^2]'); title('Joint 4 Acceleration'); set(gca,'FontSize',10);

end

%% Plotting the profiles in S - Workspace
% This profile is the same for both configurations, so we only need to plot
% it once.
figure('Name','Workspace Profiles','NumberTitle','off', ...
       'Position',[100 70 1300 720]);

% Define colors for each variable group
colors = lines(4); % X=blue, Y=orange, Z=green, phi=red

% --- Position ---
subplot(3,4,1); plot(time, px, 'LineWidth',1.5,'Color',colors(1,:)); grid on; axis tight;
xlabel('Time [s]'); ylabel('x [m]'); title('Position X'); set(gca,'FontSize',10);

subplot(3,4,2); plot(time, py, 'LineWidth',1.5,'Color',colors(2,:)); grid on; axis tight;
xlabel('Time [s]'); ylabel('y [m]'); title('Position Y'); set(gca,'FontSize',10);

subplot(3,4,3); plot(time, pz, 'LineWidth',1.5,'Color',colors(3,:)); grid on; axis tight;
xlabel('Time [s]'); ylabel('z [m]'); title('Position Z'); set(gca,'FontSize',10);

subplot(3,4,4); plot(time, pfi, 'LineWidth',1.5,'Color',colors(4,:)); grid on; axis tight;
xlabel('Time [s]'); ylabel('\phi [rad]'); title('Orientation \phi'); set(gca,'FontSize',10);

% --- Velocity ---
subplot(3,4,5); plot(time, vx, 'LineWidth',1.5,'Color',colors(1,:)); grid on; axis tight;
xlabel('Time [s]'); ylabel('v_x [m/s]'); title('Velocity X'); set(gca,'FontSize',10);

subplot(3,4,6); plot(time, vy, 'LineWidth',1.5,'Color',colors(2,:)); grid on; axis tight;
xlabel('Time [s]'); ylabel('v_y [m/s]'); title('Velocity Y'); set(gca,'FontSize',10);

subplot(3,4,7); plot(time, vz, 'LineWidth',1.5,'Color',colors(3,:)); grid on; axis tight;
xlabel('Time [s]'); ylabel('v_z [m/s]'); title('Velocity Z'); set(gca,'FontSize',10);

subplot(3,4,8); plot(time, vfi, 'LineWidth',1.5,'Color',colors(4,:)); grid on; axis tight;
xlabel('Time [s]'); ylabel('\omega_\phi [rad/s]'); title('Angular Velocity \phi'); set(gca,'FontSize',10);

% --- Acceleration ---
subplot(3,4,9); plot(time, ax, 'LineWidth',1.5,'Color',colors(1,:)); grid on; axis tight;
xlabel('Time [s]'); ylabel('a_x [m/s^2]'); title('Acceleration X'); set(gca,'FontSize',10);

subplot(3,4,10); plot(time, ay, 'LineWidth',1.5,'Color',colors(2,:)); grid on; axis tight;
xlabel('Time [s]'); ylabel('a_y [m/s^2]'); title('Acceleration Y'); set(gca,'FontSize',10);

subplot(3,4,11); plot(time, az, 'LineWidth',1.5,'Color',colors(3,:)); grid on; axis tight;
xlabel('Time [s]'); ylabel('a_z [m/s^2]'); title('Acceleration Z'); set(gca,'FontSize',10);

subplot(3,4,12); plot(time, afi, 'LineWidth',1.5,'Color',colors(4,:)); grid on; axis tight;
xlabel('Time [s]'); ylabel('\alpha_\phi [rad/s^2]'); title('Angular Acceleration \phi'); set(gca,'FontSize',10);

% Global title
sgtitle('S - Workspace Profiles','FontSize',14,'FontWeight','bold');
