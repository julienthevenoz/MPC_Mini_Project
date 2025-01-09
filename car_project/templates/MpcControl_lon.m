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
            load('tube_mpc_data.mat', 'X_tightened', 'U_tightened', 'X_f', 'R', 'P', 'Qf', 'K');


            F_tight = X_tightened.A;
            M_tight = U_tightened.A;
            f_tight = X_tightened.b;
            m_tight = U_tightened.b;
            
            obj = 0;
            con = [];
            
            u = sdpvar(nu, N-1, 'full');

            A = mpc.A;
            B = mpc.B;

            delta = sdpvar(nx, N, 'full');
            z = sdpvar(nx, N, 'full');
            v = sdpvar(nu, N-1, 'full');
            e = sdpvar(nx, N, 'full');
            x_safe = 10;

            con = [con, delta(:, 1) == x0other - x0 -  [x_safe ; 0]];

            con = [con, u(:,1) == u0];
            con = [con, v(:,1) == u0]; 
            for i = 1:N-1
                con = [con, z(:, i+1) == A* (z(:, i)) - B*(v(:,i)) ];  
                con = [con, e(:, i+1) == (A- B*K)*e(:, i) ]; %i'm very very unsure of this
                con = [con, u(:,i) == K*(delta(:,i)-z(:,1)) + v(:,1)];
             
                con = [con, M_tight*(v(:,i)) <= m_tight];
                con = [con, F_tight*(z(:,i)) <= f_tight];
                con = [con, abs(u(:,i)) <= 1];

                obj = obj + z(:,i)'*P*z(:,i) + (v(:,i))'*R*(v(:,i));
            end
            con = [con, X_f.A*(z(:,N)) <= X_f.b];
            obj = obj + z(:,N)'*Qf*z(:,N);

            debugVars = {};


            
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
            x_ref = ref;    % For clarity
            Vs_ref = x_ref;  % We have a perfect sensor so measurement = state
            us_ref = (x_ref - B*d_est - xs -A*(x_ref - xs))/B + us;  
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end

        %% COMPUTE_SP function from exercise 5.
        % Since the estimate of the disturbance change at every time-step, we
        % compute a feasible setpoint every time solving the following minimization
        % problem :
        %
        % [xsp,usp] = argmin(u)
        %             s.t. xsp = A*xsp + B*usp
        %                  rsp == C*xsp + d
        %                  umin <= usp <= umax
        %
        
    end
end
