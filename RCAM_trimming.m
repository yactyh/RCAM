clear; clc; close all;

% trimmed for straight & level flight

V_target = 85; % m/s
x = zeros(9,1);
x(1) = V_target;
u0 = zeros(5,1);
H0 = zeros(14);

xdot = RCAM_dynamics(x, u0);

V_inf = sqrt(x(1)^2 + x(2)^2 + x(3)^2);
alpha = atan2(x(3), x(1));
gamma_flightpath = x(8) - alpha;   % pitch angle minus AOA
                        % only valid at wings' level (?) what does this mean
                        % for more robust formulations for gamma
                        % see Navigation Equations

Q0 = [xdot;
    V_inf - V_target;
    gamma_flightpath;
    x(2);
    x(7);
    x(9)];


f0 = costFunc(x, u0, H0, Q0, V_target);

[zStar, f0] = fminsearch('costFunction', [x; u0], ...
    optimset('TolX',1e-10','MaxFunEvals',10000,'MaxIter',10000));

zStar

[zStar, f0] = fminsearch('costFunction', [zStar(1:9); zStar(10:14)], ...
    optimset('TolX',1e-10','MaxFunEvals',10000,'MaxIter',10000));

[zStar, f0] = fminsearch('costFunction', [zStar(1:9); zStar(10:14)], ...
    optimset('TolX',1e-10','MaxFunEvals',10000,'MaxIter',10000));

zStar