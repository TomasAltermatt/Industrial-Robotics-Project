function [V_hat, T_A, T_D] = trapezoidal(motor, dq, T_min)
    V_hat = (0.5*motor.A*T_min) - sqrt((0.5*motor.A*T_min)^2 - motor.A*dq);
    T_A = V_hat/motor.A;
    T_D = V_hat/motor.D;
    
end