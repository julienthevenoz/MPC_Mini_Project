clear; close all; clc;

%{
delta = 0.1;
u_T = 0.5;
theta = 2;
V = 10;
x=0;
y=0;

Ts = 1/10;
car = Car(Ts);
u = [delta, u_T]';
x = [x, y, theta, V]';
x_dot = car.f(x,u);
%}

Ts = 1/10;
car = Car(Ts);
Tf = 2.0;                       %% Simulation end time

x0 =[0,0, deg2rad(-2), 20/3.6]';  % (x, y, theta, V) Initial state
u  = [deg2rad(-1), 0.7]';        % (delta, u T) Constant input
% positive u_T makes it go forward
% negative u_T makes it go backward
% positive angle makes it go left
% negative angle makes it go right

params = {};                    % Setup simulation parameter struct
params.Tf = Tf; 
params.myCar.model = car;
params.myCar.x0 = x0;
params.myCar.u = u;
result = simulate(params);      % Simulate nonlinear model
visualization(car, result);

result.T       % Time at every simulation step
result.myCar.X % State trajectory
result.myCar.U % Input trajectory