function [joint_positions, joint_velocities, joint_accelerations, time_vect] = PROcubic_splines(Q, n_joints, motor)
    n_pos = length(Q(1,:));
    dQ = zeros(n_joints, n_pos-1);
    for i = 1:n_joints
        dQ(i,:) = diff(Q(i,:));
    end

    n_diff = length(dQ(1,:));
    dQ_abs = abs(dQ);
    
    %% 0) Calculate min actuating time
    
    rise_time = zeros(n_joints, n_diff);
    % this block gives all the minimum rise times for each individual movement
    % this happens for each joint (4)
    for i = 1:n_diff
        rise_time(:, i) = min_rise_time(motor, dQ_abs(:, i));
    end
    
    % Finally we take the maximum of the minimum rise times so the movements
    % are synchronized
    T_min_act = max(rise_time);
    
    %% 1) Auxiliary points transition time (initial and final Positions)
    
    % Find t1 and tend
    t1 = T_min_act(1) - sqrt(T_min_act(1)^2 - 2*dQ_abs(:,1)./motor.A);
    tend = T_min_act(end) - sqrt(T_min_act(end)^2 - 2*dQ_abs(:,end)./motor.D);
    
    %% 2) Set minimum times into a vector
    tau_vec = [];
    for i = 1:n_joints
        tau_vec = [tau_vec; T_min_act];
    end

    %% 3) Build Matrix System of Equations for splines
    a_joints = zeros(n_joints, n_pos);
    b_joints = zeros(n_joints, n_pos);
    c_joints = zeros(n_joints, n_pos);
    d_joints = Q;
    
    % We need to solve the system of eqs given by 3n-4
    
    for i = 1:n_joints
        tau = tau_vec(1, 1);
        % Generate matrix
        A = zeros(3*n_pos - 4);

        % Generate vector for equation
        sol_vec = zeros(3*n_pos - 4, 1);
        
        % Fill 1st row (exception)
        A(1, 1:2) = [tau^3, tau^2];              % Position
        A(2, 1:5) = [3*tau^2, 2*tau, 0, 0, -1];  % Velocity
        A(3, 1:5) = [6*tau, 2, 0, -2, 0];        % Acceleration
        sol_vec(1) = Q(i, 2) - d_joints(i, 1);

        % Fill remaining rows
        for idx = 2:n_pos-1
            j = idx + 1;
            sol_vec(3*(idx-1) + 1) = Q(i, j) - d_joints(i, j-1);
            
            tau = tau_vec(1, idx);
            row_end = 3*idx;
            col_end = 6*idx - 4;
            if idx ~= n_pos - 1
                A(row_end-2 : row_end, col_end-5:col_end ) =[
                    tau^3,    tau^2, tau, 0,  0,  0;
                    3*tau^2,  2*tau,   1, 0,  0, -1;
                    6*tau  ,      2,   0, 0, -2,  0];
            else
                A(row_end-2 : row_end - 1, col_end-5:col_end ) =[
                    tau^3,    tau^2, tau, 0,  0,  0;
                    3*tau^2,  2*tau,   1, 0,  0, -1];
            end
            
        end

        % Solve System
        coeffs = A\sol_vec;



    end



end