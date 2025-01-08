clear all;
close all;
clc;


Ts = 1/10; % Sample time
car = Car(Ts);
% Design MPC controller

H = 15;
mpc = NmpcControl(car, H);
% u = mpc.get_u(x0d ref); % check if the open−loop prediction is reasonable

x0 = [0 0 0 80/3.6]'; % (x, y, theta, V) 

% u0 = mpc.get_u(x0, ref2);











x0_ego = [0 0 0 80/3.6]';
x0_other = [20 0 0 80/3.6]';
ref1 = [0 80/3.6]';
ref2 = [0 100/3.6]';
params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = x0_ego;
params.myCar.u = @mpc.get_u;
params.myCar.ref = car.ref_step(ref1, ref2, 1);
params.otherCar.model = car;
params.otherCar.x0 = x0_other;
params.otherCar.u = car.u_const(80/3.6);


%%plots
% u = mpc.sol.value(mpc.U);
% x = mpc.sol.value(mpc.X);
pL = mpc.sol.value(mpc.pL);

t = tiledlayout(3,2);
nexttile
plot(pL(1,:));
title("Red car x position")
% 
% nexttile
% plot(result.myCar.X(1,:));
% title("Blue car x position")

nexttile
plot(pL(2,:));
title("Red car y position")
% 
% nexttile
% plot(result.myCar.X(2,:));
% title("Blue car y position");

%%
result = simulate(params);
visualization(car, result);