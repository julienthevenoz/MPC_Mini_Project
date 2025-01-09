close all;
clear all;

%% System initialisation

Ts = 1/10; % Sample time
car = Car(Ts);
[xs, us] = car.steady_state(120 / 3.6); 

% Design MPC controller
H_lon = 15; % Horizon length in seconds

mpc = NmpcControl(car, H_lon);

%% Control input

ref1 = [0 80/3.6]';
ref2 = [3 100/3.6]';

%% Simulation

params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = [0 0 0 80/3.6]';
params.myCar.u = @mpc.get_u;

params.myCar.ref = car.ref_step(ref1, ref2, 2);
result = simulate(params);
visualization(car, result);

