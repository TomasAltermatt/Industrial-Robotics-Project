clear all
close all
clc

% Data
motor.A = 3;
motor.D = 3;
motor.V = 4;

Q = deg2rad([0, 180; 0,90]);
dQ = diff(Q, [], 2);

%% 1) Limit case

[T_lim, dq_lim] = limit_case(motor);

%% 2) Case 1 or 2?

is_case1 = dQ < dq_lim;

%% 3) Calculate rise time
rise_time = zeros(1, length(dQ));

for i = 1:length(dQ)
    rise_time(i) = min_rise_time(motor, dQ(i));
end

%% 4) Minimum actuating time

T_min_act = max(rise_time);

%% 5) New max speed

V_hat = zeros(1, length(dQ));
T_A = V_hat;
T_D = V_hat;

for i = 1:length(dQ)
    [V_hat(i), T_A(i), T_D(i)] = trapezoidal(motor, dQ(i), T_min_act);
    
end

% Missing plot_trapezoidal