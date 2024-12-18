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

            x= sdpvar(nx,N);  %x contains [y, theta]
            u = sdpvar(nu, N-1);  % u contains [delta]

            Ad = mpc.A;
            Bd = mpc.B;

            xs = mpc.xs;
            us = mpc.us;
           
            % Constraints on stat
            % x in X = { x | Fx <= f } and we know that -0.5 <= y <= 3.5 and |theta| <= 0.0873 rad
            F = [1, 0; 
                 0, -1;
                 1, 0;
                 0, -1];
            f = [3.5; 0.5; 0.0873; 0.0873];

            %constratins on input u in U = {u | Mu <= u} and we know |delta| <= 0.5236 rad
            M = [1;
                -1];
            m = [0.5236; 0.5236];

            %stage cost is l(x,u) = Q*xT*x + R*uT*u
            Q = 10*eye(2);
            R = 1;

            %get terminal/steady_state controller (aka control law) and terminal cost 
            [K, Qf, ~] = dlqr(Ad,Bd,Q,R);  

            %let's calculate max invariant set
            % Compute maximal invariant set
            Xf = polytope([F;M*K],[f;m]);   %intersection of set given by state constraints and set given by input cstrts
            Acl = Ad+Bd*K;  %closed loop dynamics. optimal u is given by K*x -> we get x+ = (A + BK)*x
            i = 0;
            while 1
                i = i + 1;
                prevXf = Xf;
                [T,t] = double(Xf);
                preXf = polytope(T*Acl,t);
                Xf = intersect(Xf, preXf);
                if isequal(prevXf, Xf)
                    break
                end
            end
            [Ff,ff] = double(Xf);
            fprintf("Terminal / Max invariant set calculated after %i iters", i)



            %system constraints and objective variable
            con = (x(:,1) == x0) + (u(:,1) == u0); %setup initial constraints on state and input x0 u0
            obj = 0;
            
            for i=1:N-1
                xi = x(:,i); ui = u(:,i);  %define current u and x for convenience
                con = con +  (x(:,i+1) == xs + Ad*(xi - xs) + Bd*(ui - us));  %system dynamics
                con = con + (F*xi <= f) + (M*ui <= m); %constraints on acceptable states and inputs
                obj = obj + (xi-xs)'*Q*(xi-xs) + (ui-us)'*R*(ui-us);     %cost function summing
            end
            con = con + (Ff*x(:,N) <= ff) + (M*u(:,N) <= m);
            obj = obj + (x(:,N) -xs)'*Qf*(x(:,N) - xs); %last term of cost function : terminal cost Vf(X_N -xs)


            % Replace this line and set u0 to be the input that you
            % want applied to the system. Note that u0 is applied directly
            % to the nonlinear system. You need to take care of any 
            % offsets resulting from the linearization.
            % If you want to use the delta formulation make sure to
            % substract mpc.xs/mpc.us accordingly.
            input = u(:,1);
            con = con + ( u0 == input );

            % Pass here YALMIP sdpvars which you want to debug. You can
            % then access them when calling your mpc controller like
            % [u, X, U] = mpc_lat.get_u(x0, ref);
            % with debugVars = {X_var, U_var};
            debug_x = x;
            debug_u = u;
            debugVars = {debug_x, debug_u};
            
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
            xs_ref = [0; 0];
            us_ref = 0;
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
