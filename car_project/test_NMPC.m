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

u0 = mpc.get_u(x0, ref2);
u = mpc.sol.value(mpc.U);
x = mpc.sol.value(mpc.X);

t = tiledlayout(3,2);
nexttile
plot(x(2,:));
ylabel('lane y position')
xlabel('time [seconds/10]')
title('Initial open-loop MPC computation : State x(2) [y position]')
nexttile

plot(x(3,:));
ylabel('theta')
xlabel('time [seconds/10]')
title('Initial open-loop MPC computation : State x(3) [theta]')
nexttile

plot(x(4,:));
ylabel("Speed V")
xlabel('time [seconds/10]')
title('Initial open-loop MPC computation state x(4) [Speed]')
nexttile

plot(u(1,:));
ylabel('steering')
xlabel('time [seconds/10]')
title('Initial open-loop MPC computation Input u(1) over time')
nexttile

plot(u(2,:));
ylabel('throttle')
xlabel('time [seconds/10]')
title('Initial open-loop MPC computation Input u(2) over time')

%%



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