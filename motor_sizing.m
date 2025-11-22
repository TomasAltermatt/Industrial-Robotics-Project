run('dynamics_PRO.m')

%% Catalog Motor Data (ALL MOTORS WITH POWER < 200W)

% Motor Names
Motor = {'HG-MR053', 'HG-MR13', 'HG-MR23','HG-KR053', 'HG-KR13', 'HG-KR23'};

% Motor Inertia (Including brakes) [kg*m^2]
Jm = [0.0224 0.0362 0.109 0.0472 0.0837 0.243]'*10^-4;

% Motor Mass (Including brakes) [kg]
Mm = [0.54 0.74 1.3 0.54 0.74 1.3]';

% Nominal Torque [N*m]
Cn = [0.16 0.32 0.64 0.16 0.32 0.64]';

% Maximum Torque [N*m]
Cmax = [0.48 0.95 1.9 0.56 1.1 2.2]';

% Nominal rpm
rpm_n = 3000*ones(length(Cn), 1);

% Maximum rpm
rpm_max = 6000*ones(length(Cn), 1);


%% Obtain data from trajectory

Qp = joint_velocities;
Qpp = joint_accelerations;


%% Tau Limit Calculations

% Preliminary terms
Crq = rms(Fq,2);      % joint rms torque vector
Qpp_rq = rms(Qpp, 2); % joint rms acceleration vector


% alpha and beta
alpha = (Cn.^2)./Jm;
beta = 2*(Qpp_rq.*Crq + mean(Qpp.*Fq, 2));

% Get transmission ratio limits
tau_opt = zeros(n_joints, length(Jm));
tau_min = zeros(n_joints, length(Jm));
tau_max = zeros(n_joints, length(Jm));
tau_p = zeros(n_joints, length(Motor));

% Fill tau combinations for all motor-joint possibilities
for j = 1:n_joints
    for m = 1:length(Motor)
        % Check if combination is feasible for sizing
        if alpha(m) < beta(j)
            tau_opt(j, m) = NaN;
            tau_min(j, m) = NaN;
            tau_max(j, m) = NaN;
            tau_p(j, m) = NaN;
            continue
        end
        wr_max = max(Qp(j,:));
        wm_max = rpm_max(m)*30/pi;
        
        % Fill out tau vectors
        tau_opt(j, m) = sqrt(Jm(m)*Qpp_rq(j)/Crq(j));
        tau_min(j, m) = sqrt(Jm(m))*(sqrt(alpha(m) - beta(j) + 4*Qpp_rq(j)*Crq(j)) - sqrt(alpha(m) - beta(j)))/(2*Crq(j)); 
        tau_max(j, m) = sqrt(Jm(m))*(sqrt(alpha(m) - beta(j) + 4*Qpp_rq(j)*Crq(j)) + sqrt(alpha(m) - beta(j)))/(2*Crq(j));   
        tau_p(j, m) = wr_max/wm_max;
        
    end
end

% Alpha-Beta Motor Plot
motor_idx = 1:length(Motor);
figure;
for j = 1:n_joints
    subplot(2, 2, j);

    % alpha scatter
    scatter(motor_idx, alpha, 40, 'filled')
    hold on
    grid on

    % beta line
    plot([0, length(motor_idx)+1], [beta(j), beta(j)], '-', 'LineWidth',2)
    title(sprintf('$\\alpha , \\beta$  Diagram Joint %d', j), Interpreter="latex")
    xlim([0, length(Motor) + 1])

    % Axis with motor names
    xticks(motor_idx);
    xticklabels(Motor);
    xtickangle(45);

    % Axis labels
    ylabel('$\alpha, \beta [W/s]$', 'Interpreter', 'latex', 'FontSize', 14);
    xlabel('Motor', 'Interpreter', 'latex', 'FontSize', 14);
    
    % No log scale if beta = 0
    if beta(j) > 0
        set(gca, 'YScale', 'log');
    end
end

% Tau Plots
for j = 1:n_joints
    figure;
    % subplot(2, 2, j)
    hold on
    grid on
    title(sprintf('$\\tau$ Diagram, Joint %d', j), Interpreter="latex")
    scatter(motor_idx, tau_max(j, :), 40, 'blue','filled','^', 'DisplayName', ...
        '$\tau_{max}$')   % tau max
    scatter(motor_idx, tau_opt(j, :), 40, 'green','filled','o', 'DisplayName', ...
        '$\tau_{opt}$')  % tau opt
    scatter(motor_idx, tau_min(j, :), 40, 'blue','filled','v', 'DisplayName', ...
        '$\tau_{min}$')   % tau min
    scatter(motor_idx, tau_p(j, :), 50,'black', 'filled', 'hexagram','DisplayName', ...
        '$\tau_{p}$')   % tau p
    if beta(j) > 0
        set(gca, 'YScale', 'log');
    end
    % Label with motor names
    xticks(motor_idx);
    xticklabels(Motor);
    xtickangle(45);

    % Axis lim
    xlim([0, length(Motor) + 1])

    % Axis labels
    ylabel('$\tau [N \cdot m]$', 'Interpreter', 'latex', 'FontSize', 14);
    xlabel('Motor', 'Interpreter', 'latex', 'FontSize', 14);
    

    % Legend
    L = legend('show', 'Location','best');
    set(L, 'Interpreter', 'latex', 'FontSize',16)
end

%% Test a certain transmission ratio for each joint

tau = [1/100; 0.0021; 1/100; 1/100];
n_motors = length(Jm);

for j = 1:n_joints
    figure;
    for m = 1:length(Motor)
        subplot(2,3,m)
        % Get motor torques
        Cm_j = tau(j)*Fq(j,:) + Jm(m)*Qpp(j,:)/tau(j);  % motor torque vect
        Cm_j_q = rms(Cm_j);                             % motor torque RMS
        motor_rpm_vect = (Qp(j,:)/tau(j))*30/pi;
        [Cm_j_sorted, sort_idx] = sort(Cm_j, 'ascend');
        motor_rpm_vect_sorted = motor_rpm_vect(sort_idx);

        plot(motor_rpm_vect, Cm_j, 'LineWidth',1.5);
        hold on
        grid on
        plot([0, rpm_max(m)], [Cn(m),Cn(m)],'Color','g')
        plot([0, rpm_max(m)], [Cmax(m),Cmax(m)],'Color','r')
        plot([0, rpm_max(m)], [Cm_j_q,Cm_j_q])
        xlim([0, max(motor_rpm_vect)*1.2])
        legend('Cm', 'Nominal Torque', 'Maximum Catalogue Torque', 'RMS Torque')
        title(sprintf('%s, Joint %d', Motor{m}, j))
    end
end 