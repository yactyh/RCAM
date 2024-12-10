%% simulation inits
initialLongLatAlt = [0 0 50]; % m, for uav toolbox simulation
vScale = 0.2;

xScale = 1;
yScale = 1;
zScale = 1;

%% deflection constraints
% as defined in RCAM document, all radians (even the thrusters...)
% aileron
dAMax = 25*pi/180;
dAMin = -25*pi/180;

% tail
dTMax = 10*pi/180;
dTMin = -25*pi/180;

% rudder
dRMax = 30*pi/180;
dRMin = -30*pi/180;

% left thruster (1)
lThrusterMax = 10*pi/180;
lThrusterMin = 0.5*pi/180;

% right thurster (2)
rThrusterMax = 10*pi/180;
rThrusterMin = 0.5*pi/180;

%% deflection time

tThrust = 5;
