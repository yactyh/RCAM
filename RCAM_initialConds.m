% Initialise constants
clear
clc
close all

%% define constants
x0 = [85; % approx 165 knots
    0;
    0;
    0;
    0;
    0;
    0;
    0.1; % approx 5.73 deg
    0];

u = [0;
    -0.1; % approx -5.73 deg
    0;
    0.08;
    0.08];

t_final = 5;

%% run model
out = sim('RCAM_simulation.slx')

%% plot results
t = out.SimX.Time;

u = out.SimU.Data(:)
x = out.SimX.Data


figure
grid on
for i = 1:size(u)
    subplot(height(u), 1, i)
    plot(t,u(i), "b.")
    title(['u' num2str(i)])
end

for i = 1:width(x)
    if mod(i,3) == 1
        figure
        grid on
    end

    subplot(3, 1, mod(i-1,3)+1)
    plot(t,x(:,i), 'r.')
    title(['x' num2str(i)])
end
