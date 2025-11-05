function [] = plot_trapezoidal(V_hat, T_A, T_D, T_min, motor)
%% Plot
% Speed
t_vec = linspace(0, T_min, 1000);
v_vec = zeros(2, length(t_vec));

for i=1:2
    v_vec(i, :) = (t_vec<T_A(i)); % Missing all this 
end