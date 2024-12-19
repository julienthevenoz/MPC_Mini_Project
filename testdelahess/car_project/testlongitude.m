clear all;
close all;


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


%u_lon = mpc_lon.get_u(x_lon, ref_lon);%DEBUG : see what happens if we give state 80km/h and ref 120km/h
x_lon = [0 80/3.6];   %22.2 m/s
ref_lon = 120/3.6;    %33.3 m/s
%[u_lon, dx, du] = mpc_lon.get_u(x_lon', ref_lon);

x_lat = [0 80/3.6]';   
ref_lat = 3;  %on veut qu'il aille à la ref2 [3 120/3,6]
% [u_lat_0, xlat, ulat] = mpc_lat.get_u(x_lat, ref_lat);
% 
% figure
% plot(xlat(1,:));
% ylabel('lane y position')
% xlabel('time [seconds/10]')
% title('Initial open-loop MPC computation : State x(1) [y position]')
% 
% figure
% plot(xlat(2,:));
% ylabel('theta')
% xlabel('time [seconds/10]')
% title('Initial open-loop MPC computation : State x(2) [theta]')
% 
% figure
% plot(ulat(1,:));
% ylabel('steering')
% xlabel('time [seconds/10]')
% title('Initial open-loop MPC computation Input u over time')

%%


x0 = [0 0 0 80/3.6]'; % (x, y, theta, V) 
ref1 = [0 80/3.6]'; % (y ref, V ref) 
ref2 = [3 120/3.6]'; % (y ref, V ref)
params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = x0;
params.myCar.u = @mpc.get_u;
params.myCar.ref = car.ref_step(ref1, ref2, 5); % delay reference step by 5s 
result = simulate(params);
visualization(car, result);