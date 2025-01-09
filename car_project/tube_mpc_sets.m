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

u_s = us(2);


%% LQR Controller Design
% This creates a feedback controller using the LQR method. The Q and R 
% values balance how much we care about state errors versus input effort. 
% K is the controller gain, and A_cl is the new closed-loop system.

Q = 16;
R = 1;
[K, Qf, ~] = dlqr(Ad, -Bd, Q, R);
K= -K;
A_cl = Ad - (Bd * K); 

%% Robust Invariant Set
% This calculates a robust invariant set E, which represents the system's 
% safe operating bounds under feedback. Iterations stop once A_cl^i 
% (system dynamics raised to the power i) becomes negligible.

T = [1; -1];
t = [u_s + 0.5; - u_s + 0.5];

W = Polyhedron(T,t) ;
W_lifted = Bd * W;
    
E = Polyhedron.emptySet(2);
i = 1;
while true
    E_next = E + A_cl ^i * W_lifted; 
    E_next = E_next.minHRep;
    if norm(A_cl^i) < 1e-2   
        fprintf("minimum robust invariant set passed vibe check after %i iterations", i)
            break;
    end
    E = E_next;
    i = i+1;
    fprintf('iteration %i and norm %i \n',i, norm(A_cl^i));
end

    
P = dlyap(A_cl',Q + K'*R*K);

%% Tightened Constraints

x_safe = 10;
F_x = [-1 0];
f_x = -6 + x_safe ;
X = Polyhedron(F_x, f_x);   % State constraints
X_tightened = X - E;        % Tightened state constraints

F_u = [1; -1];
f_u = [1; 1];
U = Polyhedron(F_u, f_u);   % Input constraints
U_tightened = U - K * E;    % Tightened input constraints

F_tight = X_tightened.A;
M_tight = U_tightened.A;
f_tight = X_tightened.b;
m_tight = U_tightened.b;

%% Terminal Set
% This computes the terminal set (Xf), ensuring that the system remains 
% within safe bounds at the end of its trajectory. Iterations continue 
% until the terminal set converges.

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

%% PLOTS : 

% Plot for X_f
figure
plot(X_f)
legend('Xf : Terminal Set')
xlabel('Delta X')
ylabel('Delta V')
grid on

%Plot for E
figure
plot(E)
legend('E : Minimum robust invariant set')
xlabel('Delta X')
ylabel('Delta V')
grid on

save('tube_mpc_data.mat', 'X_tightened', 'U_tightened', 'X_f', 'R', 'P', 'Qf', 'K', 'x_safe');