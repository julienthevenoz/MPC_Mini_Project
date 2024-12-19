
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

A = sys_lon.A;  %est-ce qu'il faut utiliser sys_lon.A ou mpc_lon.A ? je comprends meme pas la diff entre les deux mais ils sont différents
B = sys_lon.B;
u_T_s = us(2);

%define cstrsts us - 0.5 < u_T < us + 0.5
T = [1; -1];
t = [u_T_s + 0.5; -u_T_s + 0.5];
W = Polyhedron(T,t) ; % Define disturbance set
%now we need to bring W to 2dimensions (because state space is 2d [x V], and input space is 1d [throttle], so to compare/add them in minkowsky we need
%to lift W to 2d by multiplying it with input matrix B
W_lifted = B*W;



%find the control law K that stabilizes the dynamics (i.e eigs(A + B*K) < 1) -> just use LQR controller ?
Q = 10*eye(2); %same values as usual ??
R = 1;
[K, Qf, ~] = dlqr(A, B,Q, R);



% let's find the minimum robust invariant set E (epsilon in the lecture)
%we start with initial set E = {0}
E = Polyhedron('lb', [0;0], 'ub', [0;0]); %lb and ub define lower and upper bound of a box shaped polyhedron, in this case since bounds are 0 0, it's {0} set

% max_iterations = 1000;
% for i = 1:max_iterations
%     E_next = (A + B*K) * E + W_lifted; % if E and W are Polyhedron, the operator + acts as Minkowski sum and not normal addition in MPT
%     if abs(E_next.volume - E.volume) < 1e-2 %check if diff is sufficiently small to terminate
%         fprintf("minimum robust invariant set passed vibe check after %i iterations", i)
%             break;
%     end
%     E = E_next;
% end


max_iterations = 1000;
for i = 1:max_iterations
    ABK = (A + B*K)^i;
    E_next = E + ABK*W_lifted; % if E and W are Polyhedron, the operator + acts as Minkowski sum and not normal addition in MPT
    if norm((A+ B*K)^i) < 1e-2   %check if diff is sufficiently small to terminate
        fprintf("minimum robust invariant set passed vibe check after %i iterations", i)
            break;
    end
    E = E_next;
end



if i == max_iterations
    fprintf("E didn't pass vibe check")
end


F = -1;
f = -6;
X = Polyhedron(F,f);

% Embed X into R^2
F_embedded = [-1, 0]; % Add a second dimension with 0 coefficient
f_embedded = f;

% Create the lifted Polyhedron in R^2
X_lifted = Polyhedron(F_embedded, f_embedded);

Z = X_lifted - E;

% Plot the Polyhedron
figure;
X.plot('color', 'blue', 'alpha', 0.5); % Plot X in blue with transparency
hold on;
E.plot('color', 'red', 'alpha', 0.5);  % Plot E in red with transparency
hold on;
Z.plot('color', 'green', 'alpha', 0.5);  % Plot E in red with transparency


% Customize the plot
xlabel('x');
ylabel('y');
title('Plot of Sets X and E and Z');
legend('Set X', 'Set E', 'Set Z');
grid on;