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


            Ad = mpc.A;
            Bd = mpc.B;

            % Constraints on state : None
            %Constraints on input : -1 <= u <= 1
            M =[1;-1];
            m = [1;1];

            Q= 10*eye(2);
            R=1;
            
            
            %COMPUTE TERMINAL CONTROLLER
            [K, Qf, ~]=dlqr(Ad,Bd,Q,R); %I guess 
            K=-K;
            % K:the optimal gain matrix
            % Qt:the solution of the associated algebraic Riccati equation
            % disp(K);
            
            obj = 0;
            con = [];
            
            xs = mpc.xs;
            us = mpc.us;
            con = [con, x(:,1) == x0];  %initial contraints on u0 and x0
            con = [con, u(:,1) == u0];
            obj = u(:,1)'*R*u(:,1); % Cost function


            for i = 1:N-1
                con = [con, x(:,i+1) == xs + Ad*(x(:,i)-xs) + Bd*(u(:,i)-us)];   % System dynamics
                %con = [con, F*x(:,i) <= f];                     % No State constraints :
                con = [con, M*u(:,i) <= m];                     % Input constraints
                obj = obj + (x(:,i)-xs)'*Q*(x(:,i)-xs) + (u(:,i)-us)'*R*(u(:,i)-us); % Cost function
            end
            con = [con, M*u(:,N-1) <= m]; % Terminal constraint
            obj = obj + (x(:,N)-xs)'*Qf*(x(:,N)-xs);    % Terminal weight     %I changed Q to Qf ??? does it makes sense ?

            % Replace this line and set u0 to be the input that you
            % want applied to the system. Note that u0 is applied directly
            % to the nonlinear system. You need to take care of any 
            % offsets resulting from the linearization.
            % If you want to use the delta formulation make sure to
            % substract mpc.xs/mpc.us accordingly.
            input = u(:,1);
            con = con + ( u0 == input);
            

            % Pass here YALMIP sdpvars which you want to debug. You can
            % then access them when calling your mpc controller like
            % [u, X, U] = mpc_lon.get_u(x0, ref);
            % with debugVars = {X_var, U_var};
            X_var = x; %x0-mpc.xs;
            U_var = u; %u0-mpc.us;
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
            A = mpc.A(2, 2); % déjà discrétisé dans MpcControlBase
            B = mpc.B(2, 1); % déjà discrétisé dans MpcControlBase

            % Subsystem linearization steady-state
            xs = mpc.xs(2);  %this is speed V
            us = mpc.us;     %this is throttle 

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % Vs_ref = ref;
            Vs_ref =ref; %A*(ref-xs); %correct
            us_ref = (Vs_ref - xs-A*(Vs_ref-xs))/B +us;
            %save the variables to plot the
            %us_ref = B*us;
           % us_ref =0;
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
