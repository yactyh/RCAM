function cost = costFunction(z)

x = z(1:9);
u = z(10:14);

xdot = RCAM_dynamics(x, u);

V_inf = sqrt(x(1)^2 + x(2)^2 + x(3)^2);
alpha = atan2(x(3), x(1));
gamma_flightpath = x(8) - alpha;   % pitch angle minus AOA
                        % only valid at wings' level (?) what does this mean
                        % for more robust formulations for gamma
                        % see Navigation Equations

% constraint vector:
V_target = 85;
Q = [xdot;
    V_inf - V_target;
    gamma_flightpath;
    x(2);
    x(7);
    x(9)];

H = eye(14);

cost = Q' * H * Q;