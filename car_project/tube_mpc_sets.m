close all;
clear all;
Ts = 1/10; % Sample time
car = Car(Ts);
[xs, us] = car.steady_state(120 / 3.6); 
sys = car.linearize(xs, us);
[sys_lon, ~] = car.decompose(sys);
[~, Ad, Bd, ~, ~] = Car.c2d_with_offset(sys, Ts);
H_lon = 15;

u_T_s = us(2);



mpc_lon = MpcControl_lon(sys_lon, Ts, H_lon);
A = mpc_lon.A;
B = mpc_lon.B;

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
% [K, Qf, ~] = dlqr(A, B,Q, R);
poles = [0.7, 0.8];
K = -place(A, -B, poles);

A_cl = A-B*K; %closed loop controlled system dynamics
fprintf("eigenvalues of our control system should be stable. Are they?  ");
fprintf("%g \n", eigs(A_cl));

% let's find the minimum robust invariant set E (epsilon in the lecture)
%we start with initial set E = {0}
% E = Polyhedron('lb', [0, 0], 'ub', [0, 0]); %lb and ub define lower and upper bound of a box shaped polyhedron, in this case since bounds are 0 0, it's {0} set


E = Polyhedron.emptySet(2);
i = 1;
while true
    E_next = E + A_cl^i*W_lifted; % if E and W are Polyhedron, the operator + acts as Minkowski sum and not normal addition in MPT
    E_next = minHRep(E_next);
    if norm(A_cl^i) < 1e-2   %check if diff is sufficiently small to terminate
        fprintf("minimum robust invariant set passed vibe check after %i iterations", i)
            break;
    end
    E = E_next;
    fprintf('iteration %i and norm %i \n',i, norm(A_cl^i));
    i = i+1; 
end
%%
% now that we have the min robust invariant set E, we can use it to find the tightened state and input constraints
x_safe = 6;
F = [-1 0;
     0   0];
f = [-x_safe ;
    0];
X = Polyhedron(F,f);
M = [1; -1];
m = [1; 1];
U = Polyhedron(M,m);
X_tightened = X - E;
U_tightened = U - K*E;
%%
%finally let's calculate the terminal set and terminal cost
[Q_f, ~, ~] = dare(A_cl, B, Q, R); %the terminal cost is Qf = P, the solution of  discrete-time algebraic Riccati equation:
F_tight = X_tightened.A;
f_tight = X_tightened.b;
M_tight = U_tightened.A;
m_tight = U_tightened.b;
X_f = Polyhedron([F_tight; M_tight*K], [f_tight; m_tight]);



%%
close all;

figure
plot([W_lifted E])
legend('Disturbance set W_lifted', 'min robust invariant set E')
xlabel('throttle u_T ??? not sure actually ')
ylabel('what is this axis tho ???')
grid on


figure 
plot([X X_tightened X_f])
legend("X", "tightened X", "terminal set X_f")
xlabel('what is here ? ')
ylabel('what is this axis tho ???')
grid on

figure 
plot([U U_tightened])
legend("U", "tightened U")
xlabel('throttle u_T ??? not sure actually ')
ylabel('what is this axis tho ???')
grid on
%%

save('tube_mpc_data.mat', 'X_tightened', 'U_tightened', "Q_f", "X_f")