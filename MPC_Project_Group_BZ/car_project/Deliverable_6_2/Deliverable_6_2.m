close all;
clear all;

%% System initialisation

Ts = 1/10; % Sample time
car = Car(Ts);
[xs, us] = car.steady_state(120 / 3.6); 
H = 15; % Horizon length in seconds

mpc = NmpcControl_overtake(car, H);

%% Control input

x0_ego = [0 0 0 80/3.6]';
x0_other = [20 0 0 80/3.6]';
ref1 = [0 80/3.6]';
ref2 = [0 100/3.6]';

%% Simulation

params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = x0_ego;
params.myCar.u = @mpc.get_u;
params.myCar.ref = car.ref_step(ref1, ref2, 1);
params.otherCar.model = car;
params.otherCar.x0 = x0_other;
params.otherCar.u = car.u_const(80/3.6);
result = simulate(params);
visualization(car, result);
