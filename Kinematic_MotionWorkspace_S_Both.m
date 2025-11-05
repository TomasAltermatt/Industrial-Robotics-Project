clear
close all
clc

%% Kinematics & Motion Profiles in S - Workspace


    % Length of the links
    % I remember the professor said something about a 1.4 relation between the
    % limbs of the robot, but I know is not a golden rule.
    l1 = 70;
    l2 = l1/1.4;
    l3 = l2/1.4;
    
    L=[l1; l2; l3]';
    
    % End Effector Position (Initial & Final)
    Si = [-20; -65; 85; -pi/6];
    Sf = [87; 33; 43; -pi/3];
    
    dS = Sf - Si;      % Difference of positions [xf - xo, yf - yo]
    
    
    % Now We have to split it in different motion curves for x, y, z and phi.
    Si_x = Si(1);      % Initial position in x axis (xo). Takes the first element.
    dS_x = dS(1);       % ds in the x axis (xf - xo) = dx
    
    Si_y = Si(2);      % Initial position in y axis (yo). Takes the second element.
    dS_y = dS(2);       % ds in the y axis (yf - yo) = dy
    
    Si_z = Si(3);      % Initial position in z axis (zo). Takes the third element.
    dS_z = dS(3);       % ds in the y axis (zf - zo) = dz
    
    Si_phi = Si(4);    % Initial angular position of phi. takes de fourth element.
    dS_phi = dS(4);     % ds in phi, (phi_o - phi_f) - d_phi
    
    % Defining the time profile, for botha axis lets define it the same.
    T = 9;      % Total of 9 seconds
    
    % Time instants in X, Y, Z and Phi movements.
    t1_x = 3;
    t2_x = 6;
    t3_x = T;
    
    t1_y = 3;
    t2_y = 6;
    t3_y = T;
    
    t1_z = 3;
    t2_z = 6;
    t3_z = T;
    
    t1_phi = 3;
    t2_phi = 6;
    t3_phi = T;

% Loop that goes from 1 to -1, so it shows both configurations of the Robot.
for config = 1 : -2 : -1

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
    
    % % Crear objeto de video
    % v = VideoWriter('Test_01', 'MPEG-4');
    % v.FrameRate = 24;
    % open(v);
    
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
        % this case one profile for X and another one for Y.
        res_x = Sshape(t,Si_x,dS_x,t1_x,t2_x,t3_x);
        res_y = Sshape(t,Si_y,dS_y,t1_y,t2_y,t3_y);
        res_z = Sshape(t,Si_z,dS_z,t1_z,t2_z,t3_z);
        res_phi = Sshape(t,Si_phi,dS_phi,t1_phi,t2_phi,t3_phi);
    
        % We get the results in the S space (Workspace/Cartesian)
        S = [res_x.pos; res_y.pos; res_z.pos; res_phi.pos];      % Position
        Sp = [res_x.vel; res_y.vel; res_z.vel; res_phi.vel];    % Velocity
        Spp = [res_x.acc; res_y.acc; res_z.acc; res_phi.acc];   % Acceleration
    
    
        % Solution of the position (Inverse Kinematics) - Depend on the chosen
        % configuration.
        Q = PROinv2(S, L, config);
    
        % Check one solution with the Direct Kinematics
        S_check = PROdir(Q, L);
    
        % Draw the path that the robot is performing, the first one is the one
        % we calculated based on the motion profile, the second one is the
        % calculation of the Direct Kinematics once we already have the Inverse
        % Kinematics solved. This paths should be the same.
        Path = line('XData', S(1),'YData', S(2), 'ZData', S(3), ...
            'linestyle', '-','linewidth',1,'color','#ED4588',...
            'marker','o','markersize',2,'markerfacecolor','#C79114');
        Path2 = line('XData', S_check(1),'YData', S_check(2), 'ZData', S_check(3), ...
            'linestyle', '-','linewidth',1,'color','#F59127',...
            'marker','o','markersize',2,'markerfacecolor','#C79114');
    
        % Plot the Robot in the chosen configuration
        PlotPRO(Q, L, line_color, f1, ll1); 
    
    
        % Calls the Jacobian function
        J = PROjac(Q, L);
    
        % Calculate the Velocity of the Joint Space (Qp). dQ/dt = J^(-1)*(dS/dt)
        Qp = J\Sp;
    
        % Calls the derivative of the Jacobian
        Jp = PROjacP(Q, Qp, L);     % In this case it uses only the first solution.
        
        % Calculate the Acceleration of the Joint Space (Qpp).
        Qpp = J\(Spp - Jp*Qp);
    
    
        % Save the time instant in a vector of time.
        time(i) = t;
    
        % Save the values of position, velocity and acceleration in vectors.
        % For x axis:
        px(i) = res_x.pos;
        vx(i) = res_x.vel;
        ax(i) = res_x.acc;
        % For y axis:
        py(i) = res_y.pos;
        vy(i) = res_y.vel;
        ay(i) = res_y.acc;
        % For z axis:
        pz(i) = res_z.pos;
        vz(i) = res_z.vel;
        az(i) = res_z.acc;
        % For th angle:
        pfi(i) = res_phi.pos;
        vfi(i) = res_phi.vel;
        afi(i) = res_phi.acc;
        
        % Save the values of the Jointspace in vectors.
        q1(i) = Q(1);     % Angular Position in Joint 1
        q1p(i) = Qp(1);     % Angular Velocity in Joint 1
        q1pp(i) = Qpp(1);   % Angular Acceleration in Joint 1
        
        q2(i) = Q(2);     % Angular Position in Joint 2
        q2p(i) = Qp(2);     % Angular Velcocity in Joint 2
        q2pp(i) = Qpp(2);   % Angular Acceleration in Joint 2
        
        q3(i) = Q(3);     % Angular Position in Joint 3
        q3p(i) = Qp(3);     % Angular Velcocity in Joint 3
        q3pp(i) = Qpp(3);   % Angular Acceleration in Joint 3
        
        q4(i) = Q(4);     % Angular Position in Joint base
        q4p(i) = Qp(4);     % Angular Velcocity in Joint base
        q4pp(i) = Qpp(4);   % Angular Acceleration in Joint base
        
        % Increments the iteration value
        i = i + 1;
    
    %     % Capturar frame
    %     frame = getframe(gcf);
    %     writeVideo(v, frame);
    
    end
    
    % % Cerrar el archivo de video
    % close(v);



    
    %% Plotting the profiles in Q - Joint Space
    if config == 1
        figure('Name','Joint Space Profiles - 1^{st} Configuration','NumberTitle','off', ...
               'Position',[scr(1), scr(4)/2, scr(3)/2, scr(4)/2]);

        % Global title
        sgtitle('Q - Joint Space Profiles - 1^{st} Configuration','FontSize',14,'FontWeight','bold');
    
    else
        figure('Name','Joint Space Profiles -  2^{nd} Configuration','NumberTitle','off', ...
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