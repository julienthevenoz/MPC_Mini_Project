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
Vs = 120/3.6;
[xs, us] = car.steady_state(Vs);
%g=sprintf('%d' , xs);
%fprintf('Answer: %s\n', g);
disp(sprintf('Answer of xs: (%d,%d,%d,%d)', xs)); %prints xs
disp(sprintf('Answer of us: (%d,%d)', us)); %prints us
sys = car.linearize(xs, us);

[sys_lon, sys_lat] = car.decompose(sys);
fprintf('Sys_lon, matrix A');
disp(sys_lon.A);
fprintf('Sys_lon, matrix B');
disp(sys_lon.B);
fprintf('Sys_lon, matrix C');
disp(sys_lon.C);
fprintf('Sys_lon, matrix D');
disp(sys_lon.D);


fprintf('Sys_lat, matrix A');
disp(sys_lat.A);
fprintf('Sys_lat, matrix B');
disp(sys_lat.B);
fprintf('Sys_lat, matrix C');
disp(sys_lat.C);
fprintf('Sys_lat, matrix D');
disp(sys_lat.D);