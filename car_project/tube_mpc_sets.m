close all;
clear all;
Ts = 1/10; % Sample time
car = Car(Ts);
[xs, us] = car.steady_state(120 / 3.6); 
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);
[~, Ad, Bd, ~, ~] = Car.c2d_with_offset(sys, Ts);
H_lon = 15;
%est-ce qu'il faut utiliser sys_lon.A ou les c2d with offsets ?
A = [0 Ad(1,4); 
     0 Ad(4,4)];
B = [0;
     Bd(4,2)];
u_T_s = us(2);


mpc_lon = MpcControl_lon(sys_lon, Ts, H_lon);
A = mpc_lon.A;
B = mpc_lon.B;

%define constraints us - 0.5 < u_T < us + 0.5
T = [1; -1];
t = [u_T_s + 0.5; -u_T_s + 0.5];
%t = [0.5; 0.5];
W = Polyhedron(T,t) ; % Define disturbance set
%now we need to bring W to 2dimensions (because state space is 2d [x V], and input space is 1d [throttle], so to compare/add them in minkowsky we need
%to lift W to 2d by multiplying it with input matrix B
W_lifted = B*W;

%find the control law K that stabilizes the dynamics (i.e eigs(A + B*K) < 1) -> just use LQR controller ?
Q = 20*eye(2); %same values as usual ??
R = 1;
[K, Qf, ~] = dlqr(A, B,Q, R);
K_lon= -K;

%poles = [0.7, 0.8];
%K = -place(A, -B, poles);
%K = [5 7];
A_cl = A-B*K; %closed loop controlled system dynamics
fprintf("eigenvalues of our control system should be stable. Are they?");
fprintf("%g", eigs(A_cl));



% let's find the minimum robust invariant set E (epsilon in the lecture)
%we start with initial set E = {0}
% E = Polyhedron('lb', [0, 0], 'ub', [0, 0]); %lb and ub define lower and upper bound of a box shaped polyhedron, in this case since bounds are 0 0, it's {0} set


E = Polyhedron.emptySet(2);
max_iterations = 1000;
i = 1;
while true
    E_next = E + A_cl^i*W_lifted; % if E and W are Polyhedron, the operator + acts as Minkowski sum and not normal addition in MPT
    E_next = minHRep(E_next);
    if norm(A_cl^i) < 1e-2   %check if diff is sufficiently small to terminate
        fprintf("minimum robust invariant set passed vibe check after %i iterations", i)
            break;
    end
    E = E_next;
    i = i+1;
    fprintf('iteration %i and norm %i \n',i, norm(A_cl^i));
end

    

if i == max_iterations
    fprintf("E didn't pass vibe check")
end
%%
%%% NOTRE VERSION
%{
x_safe = 10;
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


F_tight = X_tightened.A;
M_tight = U_tightened.A;
f_tight = X_tightened.b;
m_tight = U_tightened.b;


%P = dlyap(A_cl',Q + K'*R*K);
X_f = Polyhedron([F_tight;M_tight*K],[f_tight;m_tight]);
%}


%%%%% SA VERSION 
x_safe = -10;
F_x = [-1 0];
f_x = -x_safe ;
X = Polyhedron(F_x, f_x);
X_tightened = X - E;

F_u = [1; -1];
%f_u = [1 - u_T_s; 1 + u_T_s];
f_u = [1; 1];
U = Polyhedron(F_u, f_u);
U_tightened = U - K * E;

F_tight = X_tightened.A;
M_tight = U_tightened.A;
f_tight = X_tightened.b;
m_tight = U_tightened.b;

P = dlyap(A_cl',Q + K'*R*K);

X_f = Polyhedron([F_tight;M_tight*K],[f_tight;m_tight]);


i = 1;
while 1
    prevXf = X_f;
    T_Xf = X_f.A;
    t_Xf = X_f.b;
    new_Xf = Polyhedron(T_Xf*A_cl,t_Xf);
    X_f = intersect(X_f, new_Xf);
    if abs(X_f.volume - prevXf.volume) < 1e-10
        fprintf("Get our terminal set after %i iterations\n", i)
        break
    end
    fprintf("iteration %i to get our Terminal set\n",i);
    i = i + 1;
end


figure
plot([X_f])
legend('Xf : Terminal Set')
xlabel('Delta X')
ylabel('Delta V')
grid on


figure
plot([E])
legend('E : Minimum robust invariant set')
xlabel('Delta X')
ylabel('Delta V')
grid on


save('tube_mpc_data.mat', 'X_tightened', 'U_tightened', 'X_f', 'P', 'Qf', 'R', 'K_lon');