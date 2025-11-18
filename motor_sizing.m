clear all; close all; clc;
addpath('Functions')

%% Catalog Motor Data

% Motor Inertia (Including brakes) [kg*m^2]
Jm = [0.0224 0.0362 0.109 0.164 0.694 0.0472 0.0837 0.243 0.393 1.37]*10^-4;

% Nominal Torque [N*m]
Cn = [0.16 0.32 0.64 1.3 2.4 0.16 0.32 0.64 1.3 2.4];

% Maximum Torque [N*m]
Cmax = [0.48 0.95 1.9 3.8 7.2 0.56 1.1 2.2 4.5 8.4];

% Nominal rpm
rpm_n = 3000*ones(1, length(Cn));

% Maximum rpm
rpm_max = 6000*ones(1, length(Cn));


%% Obtain data from trajectory
run("dynamics_PRO.m")

% Pass trajectory velocities to rpm
Qp_rpm = Qp.*(pi/30);

%% Calculations

% Preliminary terms
Crq = rms(Fq,2);    % joint rms torque
Qp_rq = rms(Qp, 2); % joint rms velocities


% Calculate alpha and beta for sizing
alpha = (Cn.^2)./Jm;


% take arbitrary transmission ratio
tau = 1/100;

