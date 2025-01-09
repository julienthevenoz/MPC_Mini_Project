classdef NmpcControl_overtake < handle

    properties
        % The NMPC problem
        opti

        % Problem parameters
        x0, ref, x0other

        % Most recent problem solution
        sol

        % The input that you want to apply to the system
        u0

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Add any variables you would like to read to debug here
        % and then store them in the NmpcControl function below.
        % e.g., you could place X here and then add obj.X = X
        % in the NmpcControl function below.
        % 
        % After solving the problem, you can then read these variables 
        % to debug via
        %   nmpc.sol.value(nmpc.X)
        % 
        p
        X, U, pL
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    end

    methods
        function obj = NmpcControl_overtake(car, H)

            import casadi.*

            N_segs = ceil(H/car.Ts); % Horizon steps
            N = N_segs + 1;          % Last index in 1-based Matlab indexing

            nx = 4;
            nu = 2;

            % Define the NMPC optimization problem
            opti = casadi.Opti();
            
            % Parameters (symbolic)
            obj.x0 = opti.parameter(nx, 1);       % initial state
            obj.ref = opti.parameter(2, 1);       % target y, velocity
            obj.x0other = opti.parameter(nx, 1);  % initial state of other car

            % SET THIS VALUE TO BE YOUR CONTROL INPUT
            obj.u0 = opti.variable(nu, 1);
         
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            h = car.Ts;
            f_discrete  = @(x,u) RK4_car(x,u,h,@car.f);

            % Define your problem using the opti object created above

            %define decision variables
            obj.X = opti.variable(nx,N+1); % state trajectory variables
            obj.U = opti.variable(nu,N);   % control trajectory (throttle, brake)
            

            %define the cost to minimize
            cost = ...
             1000*(obj.X(4,:) - obj.ref(2))*(obj.X(4,:) - obj.ref(2))'  + ... % Minimize difference between V and V_ref
             1000*(obj.X(2,:) - obj.ref(1))*(obj.X(2,:) - obj.ref(1))' + ...  % Minimize difference between y and y_ref
             0.1*obj.U(1,:)*obj.U(1,:)' + ...                                % Minimize steering
             obj.U(2,:)*obj.U(2,:)' ;                                         % Minimize throttle

            opti.subject_to(obj.U(:,1) == obj.u0);
            opti.subject_to(obj.X(:,1) == obj.x0);
           
            for k=1:N % loop over control intervals

                opti.subject_to(obj.X(:,k+1) == f_discrete(obj.X(:,k), obj.U(:,k)));

           end
            
            opti.subject_to(-1 <= obj.U(2,:));       % Limit throttle
            opti.subject_to(obj.U(2,:)  <= 1);
            opti.subject_to(-0.5236 <= obj.U(1,:));    % Limit steering to 30 degrees 
            opti.subject_to(obj.U(1,:) <= 0.5236);
            opti.subject_to(-0.5 <= obj.X(2,:));       % Limit to stay on the rigth latitude
            opti.subject_to(obj.X(2,:)  <= 3.5 );
            opti.subject_to(-0.0873 <= obj.X(3,:));    % Limit car angle theta smaller than 5 degrees
            opti.subject_to(obj.X(3,:) <= 0.0873);

            % Define p (ego's position)
            obj.p = [obj.X(1, :); obj.X(2, :)];        % Ego's position

            % Define pL (other car's position)
            z = (0:N) * h;                             % Time vector
            pL1 = obj.x0other(1) + obj.x0other(4) * z; % Other car's x position
            pL2 = obj.x0other(2) * ones(1, N + 1);     % Other car's y position
            obj.pL = [pL1; pL2];

            % Constraints for collision avoidance
            H = [1/100, 0; 
                  0, 1/9];
    
            for k = 1:N+1
              opti.subject_to((obj.p(:, k) - obj.pL(:, k))' * H * (obj.p(:, k) - obj.pL(:, k)) >= 1); % Ellipsoidal constraint
            end
          
            opti.minimize(cost);

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Store the defined problem to solve in get_u
            obj.opti = opti;

            % Setup solver
            options = struct;
            options.ipopt.print_level = 0;
            options.print_time = 0;
            options.expand = true;
            obj.opti.solver('ipopt', options);
        end

        function u = get_u(obj, x0, ref, x0other)

            if nargin < 4
                x0other = zeros(4, 1);
            end

            % Compute solution from x0
            obj.solve(x0(1:4), ref, x0other(1:4));

            u = obj.sol.value(obj.u0);
        end

        function solve(obj, x0, ref, x0other)

            % Pass parameter values
            obj.opti.set_value(obj.x0, x0);
            obj.opti.set_value(obj.ref, ref);
            obj.opti.set_value(obj.x0other, x0other);

            obj.sol = obj.opti.solve();   % actual solve
            
            % Set warm start for next solve
            obj.opti.set_initial(obj.sol.value_variables());
            obj.opti.set_initial(obj.opti.lam_g, obj.sol.value(obj.opti.lam_g));
        end
    end
end
