H = 10;
Ts = 1/5; % Sample time
car = Car(Ts);
mpc = NmpcControl(car, H);
x0 = [0 0 0 80/3.6]';
ref = [3 100/3.6]';

params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = [0 0 0 80/3.6]'; params.myCar.u = @mpc.get_u;
ref1 = [0 80/3.6]';
ref2 = [3 100/3.6]';
params.myCar.ref = car.ref_step(ref1, ref2, 2);
result = simulate(params);
visualization(car, result);

%{
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
%}