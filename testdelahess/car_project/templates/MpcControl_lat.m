classdef MpcControl_lat < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(mpc.H/mpc.Ts); % Horizon steps
            N = N_segs + 1;              % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);

            % Initial states
            x0 = sdpvar(nx, 1);
            x0other = sdpvar(nx, 1); % (Ignore this, not used)

            % Input to apply to the system
            u0 = sdpvar(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            F=[1 0 ; 
               0 1 ; 
               -1 0 ;
               0 -1 ];
            f=[3.5;
                0.0873;
                0.5;
                0.0873];

            M=[1;-1];
            m=[0.5236;0.5236];

            A=mpc.A;
            B=mpc.B;

            xs = mpc.xs;
            us = mpc.us;

            Q= 10*eye(2);
            R=1;
            [K, Qf, ~]=dlqr(A,B,Q,R);
            K=-K;

            x = sdpvar(2,N,'full');
            u = sdpvar(1,N-1,'full');

            obj = 0;
            con = [];
             %%%%%%%%%%%%%%%%%%%%%%%%%%%
             % DELTA FORMULATION
            %{
            con = [con, x(:,1) == x0];
            for i = 1:N-1
                delta_x = x(:,i+1) - xs;
                con = [con, (x(:,i+1)-xs) == A*(x(:,i)-xs) + B*(u(:,i)-us)];   % System dynamics
                con = [con, F*(x(:,i)-xs) <= f - F*xs];                     % State constraints
                con = [con, M*(u(:,i)-us) <= m - M*us];                     % Input constraints
                obj = obj + (x(:,i)-xs)'*Qf*(x(:,i)-xs) + (u(:,i)-us)'*R*(u(:,i)-us); % Cost function
            end
            con = [con, F*(x(:,i)-xs) <= f- F*xs ];
            obj = obj + (x(:,N) - xs)'*Qf*(x(:,N) - xs);
            %}
            %%%%%%%%%%%%%%%%%%%%%%%%%%
            %NON DELTA FORMULATION
            
            con = [con, x(:,1) == x0];
            for i = 1:N-1
                con = [con, x(:,i+1) == A*x(:,i) + B*u(:,i)];   % System dynamics
                con = [con, F*x(:,i) <= f ];                     % State constraints
                con = [con, M*u(:,i) <= m ];                     % Input constraints
                obj = obj + (x(:,i)-xs)'*Qf*(x(:,i)-xs) + (u(:,i)-us)'*R*(u(:,i)-us); % Cost function
            end
            
            con = [con, F*x(:,N) <= f ];
            obj = obj + (x(:,N) - xs)'*Qf*(x(:,N) - xs);
            
            % Replace this line and set u0 to be the input that you
            % want applied to the system. Note that u0 is applied directly
            % to the nonlinear system. You need to take care of any 
            % offsets resulting from the linearization.
            % If you want to use the delta formulation make sure to
            % substract mpc.xs/mpc.us accordingly.
            %input = 0.5;
            input = u(:,1);
            con = con + ( u0 == input );

            % Pass here YALMIP sdpvars which you want to debug. You can
            % then access them when calling your mpc controller like
            % [u, X, U] = mpc_lat.get_u(x0, ref);
            % with debugVars = {X_var, U_var};
            debug_u = u;
            debug_x = x;
            debugVars = {x,u};
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {x0, x_ref, u_ref, x0other}, {u0, debugVars{:}});
        end
        
        % Computes the steady state target which is passed to the
        % controller
        function [xs_ref, us_ref] = compute_steady_state_target(mpc, ref)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Steady-state system
            A = mpc.A;
            B = mpc.B;


            % Linearization steady-state
            xs = mpc.xs;
            us = mpc.us;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %xs_ref = [0; 0];
            
            if ~(ref==0)
                fprintf('not zero');
            end


            xs_ref = [ref;0];
            %us_ref_1 = (xs_ref - xs - A*(xs_ref-xs));
            %us_ref = us_ref_1(1)/B(1)+us;
            us_ref = ((1-A(1,1))*(xs_ref(1)-xs(1))-A(1,2)*(xs_ref(2)-xs(2)))/B(1) + us;
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
