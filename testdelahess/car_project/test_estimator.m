close all;
clear all;
clc


Ts = 1/10; % Sample time
car = Car(Ts);
[xs, us] = car.steady_state(120 / 3.6); 
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);
% Design MPC controller
H_lon = 15; % Horizon length in seconds
% Get control input for longitudinal subsystem

mpc_lon = MpcControl_lon(sys_lon, Ts, H_lon);
mpc_lat = MpcControl_lat(sys_lat, Ts, H_lon);
mpc = car.merge_lin_controllers(mpc_lon, mpc_lat);


estimator = LonEstimator(sys_lon, Ts);
x0 = [0 0 0 80/3.6]'; % (x, y, theta, V)
ref1 = [0 80/3.6]'; % (y ref, V ref)
ref2 = [3 50/3.6]'; % (y ref, V ref)
params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = x0;
params.myCar.est_fcn = @estimator.estimate;
params.myCar.est_dist0 = 0;
params.myCar.u = @mpc.get_u;
params.myCar.ref = car.ref_step(ref1, ref2, 2); % delay reference step by 2s;
result = simulate(params);
%%


%%%plot the estimations
figure
plot(result.myCar.Z_hat(4,1:11));
hold on;
plot(result.myCar.X(4,:));
ylabel('V_hat estimation [m/s]')
xlabel('time [seconds/10]')
title('Real speed vs estimated speed ')
legend('V_hat','V')
hold off;

figure
plot(result.myCar.Z_hat(5,1:10));
ylabel('disturbance d')
xlabel('time [seconds/10]')
title('Estimated disturbance')

% figure
% plot(xlat(2,:));
% ylabel('theta')
% xlabel('time [seconds/10]')
% title('Initial open-loop MPC computation : State x(2) [theta]')


%%
visualization(car, result);