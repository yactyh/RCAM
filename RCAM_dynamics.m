%% RCAM dynamical system equations
% 9 states (v_x,y,z ; omega_x,y,z ; euler angles)
% 5 inputs (deflections aileron,tail,rudder ; thrusters l&r)
% parameters as specified in the RCAM document
function xdot  = RCAM_dynamics(x, u)

% ----- BODY CONSTANTS -----
m = 120000; % kg, AC total mass

cbar = 6.6;     % m, mean aero chord
lt = 24.8;      % m, distance of ACs between tail and body
S = 260;        % m^2, wing planform area
St = 64;        % m^2, tail planform area

xCG = [0.23*cbar; 0; 0.10*cbar];    % m, CG position in 
xAC = [0.12*cbar; 0; 0];            % m, AC position

% ----- ENGINE LOCATIONS -----
xEngL = [0; -7.94; -1.9];           % m, left thrust force position
xEngR = [0; 7.94; -1.9];            % m, right thrust force position

% ----- WORLD PARAMS -----
rho = 1.225;                % kg/m^3, air density at sea level
g = 9.81;                   % m/s^2, accel due to gravity
dEpsdAlpha = 0.25;          % rad/rad, change in downwash w.r.t. alpha
alpha_L0 = -11.5*pi/180;    % rad, zero-lift AOA
CL_alpha = 5.5;             % N/rad (?) linear region lift slope
a0 = 15.212;                % coeff for nonlinear region, differs from RCAM
a = [-155.2; 609.2; -768.5];% coeffs for nonlinear (cubic) region lift,
alpha_s = 14.5*(pi/180);    % rad, alpha where behaviour switches to nonlinear

% ----- CONTROL LIMITS & SATURATION -----
% to be enforced in Simulink

% ----- INTERMEDIATE VARIABLES -----
V_inf = sqrt(x(1)^2 + x(2)^2 + x(3)^2);  % m/s, airspeed

alpha = atan2(x(3), x(1));  % rad, AOA
beta = asin(x(2)/V_inf);      % rad, sideslip

q = 0.5*rho*V_inf^2;        % Pa, dynamic pressure

omega_be_b = [x(4); x(5); x(6)];  % rad/s, angular vel w.r.t body frame
V_b = [x(1); x(2); x(3)];         % m/s, velocity of body

% ----- AERODYNAMIC FORCE COEFFICIENTS -----
% LIFT: overall = wing-body + tail

if alpha <= alpha_s
    CL_wb = CL_alpha * (alpha - alpha_L0);
else
    alphaPowers = [1; alpha; alpha^2; alpha^3];
    CL_wb = dot([a0; a], alphaPowers);
end

epsilon = dEpsdAlpha * (alpha - alpha_L0);
alpha_t = alpha - epsilon + u(2) + 1.3*x(5)*lt/V_inf;
CL_t = 3.1 * alpha_t * (St/S);

CL = CL_wb + CL_t;

% SIDEFORCE:
CY = -1.6*beta + 0.24*x(3);

% DRAG:
CD = 0.13 + 0.07*(5.5*alpha + 0.654)^2;


% ----- DIMENSIONAL AERODYNAMIC FORCES -----
FA_s = [-CL*q*S; CY*q*S; -CD*q*S];  % stability axis
RotY = [cos(alpha) 0 -sin(alpha);
        0 1 0;
        sin(alpha) 0 cos(alpha);];
FA_b = RotY * FA_s;                 % body axis
% FA_w = rotz(beta) * FA_s;           % wind axis

% ----- AERODYNAMIC MOMENT COEFFS ABOUT AC -----
eta = [-1.4*beta; 
       -0.59-3.1*St*lt/(S*cbar)*(alpha-epsilon);
       (1-alpha*180/(15*pi)).*beta];

CM_omega = (cbar*V_inf) * [-11, 0, 5;
                           0, -4.03*St*lt^2/(S*cbar), 0;
                           1.7, 0, -11.5];
CM_u = [-0.6 0 0.22;
        0 -3.1*St*lt/(S*cbar) 0;
        0 0 -0.63];

CM_AC_b = eta + CM_omega * omega_be_b + CM_u * [u(1); u(2); u(3)]; % body


% ----- DIMENSIONAL MOMENTS -----
MA_AC_b = cbar*q*S*CM_AC_b;
MA_CG_b = MA_AC_b + cross(FA_b, (xCG - xAC));

% ----- PROPULSION -----
F_E1_b = [u(4) * m * g; 0; 0];
F_E2_b = [u(5) * m * g; 0; 0];
FE_b = F_E1_b + F_E2_b;

mu1 = [xCG(1) - xEngL(1);       % displacement to CG of engine 1
       -(xCG(2) - xEngL(2));
       xCG(3) - xEngL(3)]; 
mu2 = [xCG(1) - xEngR(1);       % displacement to CG of engine 2
       -(xCG(2) - xEngR(2));
       xCG(3) - xEngR(3)]; 

M_CG_E1_b = cross(mu1, F_E1_b);
M_CG_E2_b = cross(mu2, F_E2_b);

M_CG_E_b = M_CG_E1_b + M_CG_E2_b;
M_CG_b = M_CG_E_b + MA_CG_b;    % total moment about CG (engines + aero)

% ----- GRAVITY EFFECTS -----
g_b = [-g*sin(x(8));
       g*cos(x(8))*sin(x(7));
       g*cos(x(8))*cos(x(7))];
Fg_b = m*g_b;


% ----- INERTIA MATRIX -----
Ib = m*[40.07 0 -2.0923;
        0 64 0;
        -2.0923 0 99.92];

IbInv = 1/m * [0.0249836 0 0.0005231512910;
               0 0.015625 0;
               0.000523151 0 0.010018961];

F_b = FE_b + Fg_b + FA_b;

accels = 1/m * F_b - cross(omega_be_b, V_b);
angleAccels = IbInv * (M_CG_b - cross(omega_be_b, Ib * omega_be_b));

H = [1 sin(x(7))*tan(x(8)) cos(x(7))*tan(x(8));
     0 cos(x(7)) -sin(x(8));
     0 sin(x(7))/cos(x(8)) cos(x(7))/cos(x(8))];
angleRates = H * omega_be_b;

xdot = [accels; angleAccels; angleRates];