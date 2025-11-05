function [T_rise] = min_rise_time(motor, dq)
[T_lim, dq_lim] = limit_case(motor);
T_rise = zeros(1, length(motor.A));

for ii = 1: length(dq_lim)
    A = motor.A(ii);
    D = motor.D(ii);
    V = motor.V(ii);
    if dq(ii) < dq_lim(ii)
        T_rise(ii) = ((sqrt(A/D) + sqrt(D/A)))*sqrt(2*dq(ii)*1/(A + D));
    else
        T_rise(ii) = dq(ii)/V + V*(A + D)/(2*A*D);
    end
end

end