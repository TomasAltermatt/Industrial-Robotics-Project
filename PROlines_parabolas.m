function [joint_positions, joint_velocities, joint_accelerations, time_vect] = PROlines_parabolas(Q, n_joints, motor)
    % Missing velocity/acceleration as well
   
    % Adapt based on desired Workspace Positions, not arbitrarily
    % Each q represents a joint, each entry represents a position
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
    
    %% 2) Internal parabolic transition time t_j
    min_act_time_joints = [];
    for i = 1:n_joints
        min_act_time_joints = [min_act_time_joints; T_min_act];
    end
    
    
    % We first subtract the t1 and tend from start/end points to obtain minimum
    % time between auxiliary points and the next points
    min_act_time_joints(:, 1) = T_min_act(:, 1) - 0.5*t1;
    min_act_time_joints(:, end) = T_min_act(:, end) - 0.5*tend;
    
    % Obtain velocity vectors
    % each entry is divided by same index entry 
    % in other matrix Q(1,1)/t(1,1) and so on
    q_dot = dQ./min_act_time_joints; 
    % here we obtain time as tj = (vel_j - vel_(j-1))/acc_j 
    t_j = abs(diff(q_dot, [], 2)./motor.A);
    
    % now we append the start and end times
    t_j = [t1, t_j, tend];
    
    %% 3) Time vector
    
    dt_j = zeros(n_joints, length(min_act_time_joints(1,:)));
    for i = 1:length(min_act_time_joints(1,:))
        dt_j(:, i) = min_act_time_joints(:, i) - 0.5*(t_j(:, i) + t_j(:,i+1));
    end
    
    dt_j = [zeros(n_joints, 1), dt_j];
    
    t_vec = zeros(n_joints, length(t_j(1,:)) + length(dt_j(1,:)));
    for i = 1:length(t_j(1,:))
        t_vec(:, i*2-1) = dt_j(:, i);
        t_vec(:, i*2) = t_j(:, i);
    end
    
    t_vec = cumsum(t_vec, 2);
   
    
    %% 4) Line coefficients
    t_line_vec = cumsum([t1/2, min_act_time_joints], 2);
    
    al = q_dot;
    bl = Q(:, 2:end) - al.*t_line_vec(:, 2:end);
    
    %% 5) Parabola coefficients
    q_dot_ext = [zeros(n_joints, 1), q_dot, zeros(n_joints, 1)];
    Q_end_parabola = [al, al(:, end)].*t_vec(:, 2:2:end)+[bl, bl(:,end)];
    
    ap = diff(q_dot_ext, [], 2)./(2*t_j);
    ap(isnan(ap)) = 0;
    
    bp = [q_dot, zeros(n_joints, 1)] - ap*2.*t_vec(:, 2:2:end);
    bp(isnan(bp)) = 0;
    
    cp = (Q_end_parabola - ap.*t_vec(:, 2:2:end).^2 - bp.*t_vec(:, 2:2:end));
    cp(:,end) = Q(:,end)- ap(:,end).*t_vec(:,end).^2 - bp(:,end).*t_vec(:,end);
    cp(isnan(cp)) = 0;
    
    
    %% 6) Get trajectory
    % Generate time vector for plot
    n_pairs = 0.5*length(t_vec(1,:)) - 1;
    
    % Calculate joint positions over time using the coefficients
    
    step = 1e-4;
    time_vect = 0:step:max(t_vec(:));
    joint_positions = zeros(n_joints, length(time_vect)); % Initialize joint_positions to store the trajectory
    joint_velocities = zeros(n_joints, length(time_vect));
    joint_accelerations = zeros(n_joints, length(time_vect));
    
    for n = 1:n_joints
        t_vec_joint = t_vec(n,:);
        for i = 1:2*n_pairs + 1
            start = t_vec_joint(i);
            finish = t_vec_joint(i+1);
            for j = 1:length(time_vect)
                t = time_vect(j);
                
                % If i'm not in the interval i go next iteration
                if t < start || t > finish
                    continue
                end

                % line case
                if mod(i, 2) == 0
                    idx = i/2;
                    joint_positions(n, j) = al(n, idx).*t + bl(n, idx);
                    joint_velocities(n, j)= al(n, idx);
                    joint_accelerations(n, j) = 0;

                % parabola case
                else
                    idx = (i+1)/2;
                    joint_positions(n, j) = ap(n, idx).*(t^2) + bp(n, idx).*t ...
                        +cp(n, idx);
                    joint_velocities(n, j) = 2*ap(n, idx).*t + bp(n, idx);
                    joint_accelerations(n, j) = 2*ap(n, idx);
                end                
            end
        end
    end


end

