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

t_final = 180;

%% run model
out = sim('RCAM_simulation.slx')

%% plot results
t = out.SimX.Time;

figure
grid on
for i = 1:5
    subplot(height(u), 1, i)
    plot(t,out.SimU.Data(:,i), "b.")
    title(['u' num2str(i)])
end

for i = 1:9
    if mod(i,3) == 1
        figure
        grid on
    end

    subplot(3, 1, mod(i-1,3)+1)
    plot(t,out.SimX.Data(:,i), 'r.')
    title(['x' num2str(i)])
end
