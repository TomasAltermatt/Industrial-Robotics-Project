clear all; close all; clc;
addpath('Functions')

%% --- Setup Input Ranges and Parameters ---
% Define angle limits
alpha_lims = [0, pi/2];
beta_lims = [0, pi/2];
gamma_lims = [0, pi/2];
% theta_lims is not used for this X-Z plot as theta_0 is fixed
theta_0 = 0; 

% Define the resolution (number of points for each joint)
N_points = 10;
alpha_range = linspace(alpha_lims(1), alpha_lims(2), N_points);
beta_range = linspace(beta_lims(1), beta_lims(2), N_points);
gamma_range = linspace(gamma_lims(1), gamma_lims(2), N_points);

% Define joint lengths
l1 = 50e-2;
l2 = l1/1.4;
l3 = l2/1.4;
L = [l1, l2, l3];

%% --- 1. Generate All Combinations (Vectorized Setup) ---
% Use ndgrid to get all combinations of the three joints
[A_grid, B_grid, G_grid] = ndgrid(alpha_range, beta_range, gamma_range);

% Flatten the grids into column vectors
alpha_all = A_grid(:);
beta_all  = B_grid(:);
gamma_all = G_grid(:);
N = length(alpha_all); % Total number of configurations

% Assemble the full input matrix Q_act_all (4 x N matrix)
% The fourth row is a constant theta_0
Q_act_all = [alpha_all, beta_all, gamma_all, repmat(theta_0, N, 1)].'; % Transpose to (4 x N)

%% --- 2. Compute Workspace Position using arrayfun ---
% arrayfun applies PROdir to each column (i.e., each Q_act vector)
% 'UniformOutput', false is used because PROdir returns a vector, not a scalar.
S_cell = arrayfun(@(i) PROdir(Q_act_all(:,i), L), 1:N, 'UniformOutput', false);

% Convert the cell array of output vectors back into a single matrix
S_all = cell2mat(S_cell); % S_all is now a (3 x N) matrix

% Extract workspace coordinates (x and z)
x_vec = S_all(1, :).'; % First row (x-coordinates), transposed to be a column vector
z_vec = S_all(3, :).'; % Third row (z-coordinates), transposed to be a column vector

%% --- 3. Plotting and Centering ---
figure;
scatter(x_vec, z_vec,'filled', 'LineWidth', 0.25);
grid on;
xlabel('r [m]')
ylabel('z [m]')

% Center the plot axes at the origin
centerOrigin(gca);

%% --- Helper Function (Center Axis) ---
% NOTE: In a professional setup, this function would be in a separate .m file.
function centerOrigin(ax)
    % Sets the axes of the plot specified by handle 'ax' to intersect at (0,0).
    
    if nargin < 1
        ax = gca;
    end
    
    % Ensure symmetric limits for true centering
    max_x = max(abs(ax.XLim));
    max_y = max(abs(ax.YLim));
    xlim(ax, [-max_x, max_x]);
    ylim(ax, [-max_y, max_y]);
    
    % Adjust tick directions for clarity with centered axes
    ax.TickDir = 'in'; 
end