classdef MpcControl_lon < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   V_ref, u_ref - reference state/input
            %   d_est        - disturbance estimate
            %   x0other      - initial state of other car
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(mpc.H/mpc.Ts); % Horizon steps
            N = N_segs + 1;              % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets
            V_ref = sdpvar(1);
            u_ref = sdpvar(1);

            % Disturbance estimate (Ignore this before Todo 4.1)
            d_est = sdpvar(1);

            % Initial states
            x0 = sdpvar(nx, 1);
           
     
            x0other = sdpvar(nx, 1); % (Ignore this before Todo 5.1)

            % Input to apply to the system
            u0 = sdpvar(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system.
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            x= sdpvar(nx,N);
            u = sdpvar(nu, N-1);


            A = mpc.A;
            B = mpc.B;
            % Constraints on state
            % x in X = { x | Fx <= f }

            %{
            F = [1 0 0; 
                 0 1 0; 
                -1 0 0; 
                 0 -1 0]; 

            f = [0.5; 0.0873; 3.5; 0.0873];
            %}

            %F = [0; 1];
            %f=[0;0];


            % Constraints on control input
            % u in U = { u | Mu*u <= mu }
            %{
            M = [1 0;
                0 1;
                -1 0;
                0 -1];
            m = [1; 0.5236; 1; 0.5236];
            %}
            M =[1;-1];
            m = [1;1];

            %N=10;
            %no idea where we get this info but
            Q= 10*eye(2);
            R=1;
            
            
            %COMPUTE TERMINAL CONTROLLER
            [K, Qf, ~]=dlqr(A,B,Q,R); %I guess 
            K=-K;
            % K:the optimal gain matrix
            % Qt:the solution of the associated algebraic Riccati equation
            disp(K);
            
            %COMPUTE SETS AND WEIGHTS WITH CODE FROM LAST WEEK
            %P = polytope(H,h); %creates the polytope {x|Hx<=h}
            %h6=plot(polytope(F,f), 'b'); %blue
            %hold on;
            
            Xf=polytope([M*K],[m]); %Hs of exercise 3 are replaced by Fs
            Acl=[A+B*K];
            
            h7=plot(Xf, 'c');
            hold on;
            i = 1;
            
            while 1
                Xf_prev = Xf;
                [P,p]=double(Xf); % initiates F matrix and f vector with defining matrix and vector of Xf
                preXf = polytope(P*Acl,p);
                
                Xf=intersect(Xf, preXf);%new polytope which is the intersection with the preset
                if Xf == Xf_prev
                    break;
                end
            
                h4=plot(Xf, 'y'); 
	            fprintf('Iteration %i... not yet equal\n', i)
	            
            
	            i = i + 1;
            end
            
            fprintf('Maximal invariant set computed after %i iterations\n\n', i);
            h5=plot(Xf,'g');
            legend([h5;h4;h7],{'Invariant set';'State constraints';'Iterations'});
           
            [Ff,ff] = double(Xf); % initiates Ff and ff vector with defining matrix and vector of Xf


            obj = 0;
            con = [];

            for i = 1:N-1
                con = [con, x(:,i+1) == A*x(:,i) + B*u(:,i)];   % System dynamics
                %con = [con, F*x(:,i) <= f];                     % State constraints
                con = [con, M*u(:,i) <= m];                     % Input constraints
                obj = obj + x(:,i)'*Q*x(:,i) + u(:,i)'*R*u(:,i); % Cost function
            end
            con = [con, Ff*x(:,N) <= ff]; % Terminal constraint
            obj = obj + x(:,N)'*Qf*x(:,N);    % Terminal weight

            % Replace this line and set u0 to be the input that you
            % want applied to the system. Note that u0 is applied directly
            % to the nonlinear system. You need to take care of any 
            % offsets resulting from the linearization.
            % If you want to use the delta formulation make sure to
            % substract mpc.xs/mpc.us accordingly.
            con = con + ( u0 == 0 );

            % Pass here YALMIP sdpvars which you want to debug. You can
            % then access them when calling your mpc controller like
            % [u, X, U] = mpc_lon.get_u(x0, ref);
            % with debugVars = {X_var, U_var};
            X_var = x0-mpc.xs;
            U_var = u0-mpc.us;
            debugVars = {X_var, U_var};
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {x0, V_ref, u_ref, d_est, x0other}, {u0, debugVars{:}});
        end
        
        % Computes the steady state target which is passed to the
        % controller
        function [Vs_ref, us_ref] = compute_steady_state_target(mpc, ref, d_est)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            %   d_est  - disturbance estimate (Ignore before Todo 4.1)
            % OUTPUTS
            %   Vs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Steady-state subsystem
            A = mpc.A(2, 2);
            B = mpc.B(2, 1);

            % Subsystem linearization steady-state
            xs = mpc.xs(2);
            us = mpc.us;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            Vs_ref = 0;
            us_ref = 0;
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
