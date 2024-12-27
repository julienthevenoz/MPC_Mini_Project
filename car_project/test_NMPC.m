clear all;
close all;



Ts = 1/10; % Sample time
car = Car(Ts);
% Design MPC controller

H = 15;
mpc = NmpcControl(car, H);
% u = mpc.get_u(x0d ref); % check if the open−loop prediction is reasonable



x0 = [0 0 0 80/3.6]'; % (x, y, theta, V) 
ref1 = [0 80/3.6]'; % (y ref, V ref) 
ref2 = [3 120/3.6]'; % (y ref, V ref)

params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = [0 0 0 80/3.6]';
params.myCar.u = @mpc.get_u;
ref1 = [0 80/3.6]';
ref2 = [3 100/3.6]';
params.myCar.ref = car.ref_step(ref1, ref2, 2);
result = simulate(params);
visualization(car, result);