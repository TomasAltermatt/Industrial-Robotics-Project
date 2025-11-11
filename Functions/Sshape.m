function res = Sshape(t, S0, dS, t1, t2, t3)

% S-shape Motion Curve:
% This function is capable to calculate the velocity, acceleration and
% position for a especific instant (t in this case).

% Input t, is a time instant from zero to the end of the movement.
% Input S0, original position, starting point.
% Input dS, total rise, the space that we have to perform. So I start in S
% and I arrive en S + dS.
% Input t1, time in which there is the transition from positive to nil
% acceleration.
% Input t2, time in which there is the translation and still nil
% acceleration, constant velocity.
% Input t3, time in which there is the transition from nil to negative
% acceleration.


% Calculate Velocity, Aceleration and Deceleration
V = (dS)/(t1/2 + (t2-t1) + (t3-t2)/2);
A = V/t1;
D = V/(t3-t2);

% Here the outputs are calculated.
if t<t1
    res.pos = S0 + 1/2*A*t^2;               % The behaviour of position with a constant acceleration
    res.vel = A*t;                          % A linear behaviour of velocity
    res.acc = A;                            % Constant acceleration
elseif t<t2
    res.pos = S0 + 1/2*A*t1^2 + V*(t-t1);   % Position
    res.vel = V;                            % Constant Velocity
    res.acc = 0;                            % Nil acceleration
else
    res.pos = S0 + dS - D*(t3-t)^2/2;       % Position
    res.vel = V - D*(t - t2);               % Linear Velocity
    res.acc = -D;                           % Constant deceleration
end

% Output is a structure 'res' with handlers pos, vel and acc.

end