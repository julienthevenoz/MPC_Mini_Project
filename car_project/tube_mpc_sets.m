close all;
clear all;

%% System initialisation

Ts = 1/10; % Sample time
car = Car(Ts);
[xs, us] = car.steady_state(120 / 3.6); 
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);
sysd = c2d(sys_lon, Ts);
[Ad,Bd,~,~] = ssdata(sysd);


u_T_s = us(2);

% mpc_lon = MpcControl_lon(sys_lon, Ts, H_lon);
% A = mpc_lon.A;
% B = mpc_lon.B;

% Define us - 0.5 < u_T < us + 0.5


%find the control law K that stabilizes the dynamics (i.e eigs(A + B*K) < 1) -> just use LQR controller ?
Q = 16; %same values as usual ??
R = 1;
[K, Qf, ~] = dlqr(Ad, -Bd, Q, R);
K= -K;

A_cl = Ad - (Bd * K); %closed loop controlled system dynamics
fprintf("eigenvalues of our control system should be stable. Are they?");
fprintf("%g", eigs(A_cl));

T = [1; -1];
t = [u_T_s + 0.5; - u_T_s + 0.5];

W = Polyhedron(T,t) ;
W_lifted = Bd * W;
    
E = Polyhedron.emptySet(2);
i = 1;
while true
    E_next = E + A_cl ^i * W_lifted; % if E and W are Polyhedron, the operator + acts as Minkowski sum and not normal addition in MPT
    E_next = E_next.minHRep;
    if norm(A_cl^i) < 1e-2   %check if diff is sufficiently small to terminate
        fprintf("minimum robust invariant set passed vibe check after %i iterations", i)
            break;
    end
    E = E_next;
    i = i+1;
    fprintf('iteration %i and norm %i \n',i, norm(A_cl^i));
end

    
P = dlyap(A_cl',Q + K'*R*K);

x_safe = 10;
F_x = [-1 0];
f_x = -6 + x_safe ;
X = Polyhedron(F_x, f_x);
X_tightened = X - E;

F_u = [1; -1];
f_u = [1; 1];
U = Polyhedron(F_u, f_u);
U_tightened = U - K * E;

F_tight = X_tightened.A;
M_tight = U_tightened.A;
f_tight = X_tightened.b;
m_tight = U_tightened.b;

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
plot(X_f)
legend('Xf')
xlabel('??')
ylabel('???')
grid on



figure
plot([E W_lifted ])
legend('Disturbance set W_lifted', 'min robust invariant set E')
xlabel('throttle u_T ??? not sure actually ')
ylabel('what is this axis tho ???')
grid on



figure
plot([U_tightened])
legend('U_T')
xlabel('throttle u_T ??? not sure actually ')
ylabel('what is this axis tho ???')
grid on

save('tube_mpc_data.mat', 'X_tightened', 'U_tightened', 'X_f', 'R', 'P', 'Qf', 'K');