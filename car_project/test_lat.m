Ts = 1/10; % Sample time
car = Car(Ts);
[xs, us] = car.steady_state(120 / 3.6); sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);
% Design MPC controller
H_lon = 10; % Horizon length in seconds
mpc_lat = MpcControl_lat(sys_lat, Ts, H_lon);
% Get control input for longitudinal subsystem

u_lat = mpc_lat.get_u(x_lat, ref_lat);