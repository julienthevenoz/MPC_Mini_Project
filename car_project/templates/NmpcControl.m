classdef NmpcControl < handle

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
        X, U, pL, p
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    end

    methods
        function obj = NmpcControl(car, H)

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
            f_discrete  = @(x,u) RK4(x,u,h,@car.f);

            % Define your problem using the opti object created above

             %define decision variables
            obj.X = opti.variable(nx,N+1); % state trajectory variables
            obj.U = opti.variable(nu,N);   % control trajectory (throttle, brake)
            obj.p = opti.variable(2, N+1);  %define p, the [x and y] of blue car (yes it's superfluous I know)
            %obj.u0 = obj.U(:,1);
            obj.pL = opti.variable(2,N+1); %pL is [x,y] of red car
            pL0 = [obj.x0other(1) ; obj.x0other(2)]; %initial state of red car
            V_red = obj.x0other(4); %speed of the red car (it is constant)
            

             %define the cost to minimize
            cost = ...
             100*(obj.X(4,:) - obj.ref(2))*(obj.X(4,:) - obj.ref(2))'  + ... % minimize difference between V and V_ref
             100*(obj.X(2,:) - obj.ref(1))*(obj.X(2,:) - obj.ref(1))' + ... %minimize difference between y and y_ref
             0.01*obj.U(1,:)*obj.U(1,:)' + ... % Minimize steering
             obj.U(2,:)*obj.U(2,:)' ;% Minimize throttle;

            % change this line accordingly
            %opti.subject_to( obj.u0 == 0 );

            opti.subject_to(obj.U(:,1) == obj.u0);
            opti.subject_to(obj.X(:,1) == obj.x0);
            opti.subject_to(obj.pL(1,1) == pL0(1));  %initial x of red car is given 
            opti.subject_to(obj.pL(2,:) == pL0(2));  %initial and all future y of red car are given (since red car doesn't change lane)
            opti.subject_to(obj.p(1,:) == obj.X(1,:));  %define that p is simply x and y of blue car
            opti.subject_to(obj.p(2,:) == obj.X(2,:));
           

            for k=1:N % loop over control intervals
  
                 opti.subject_to(obj.X(:,k+1) == f_discrete(obj.X(:,k), obj.U(:,k)));
                 opti.subject_to(obj.pL(1,k+1) == obj.pL(1,k) + V_red * h);  %predict the position x of red car by integration  %NB the y position of red car never changes
                 
                % p = [obj.X(1,k) ; obj.X(2,k)];
                % obj.pL = obj.pL + [obj.x0other(1) + obj.x0other(4)*h ; obj.x0other(2)];

            end


            H = [1/100 , 0 ;
                0, 1/9 ];
            
            opti.subject_to(-1 <= obj.U(2,:));  %limit throttle
            opti.subject_to(obj.U(2,:)  <= 1);
            opti.subject_to(-0.5236 <= obj.U(1,:)); %limit steering to 30 degrees 
            opti.subject_to(obj.U(1,:) <= 0.5236);
            opti.subject_to(-0.5 <= obj.X(2,:)); % car mustn't go out of the road
            opti.subject_to(obj.X(2,:)  <= 3.5 );
            opti.subject_to(-0.0873 <= obj.X(3,:));  %car angle theta must be smaller than 5 degrees
            opti.subject_to(obj.X(3,:) <= 0.0873);
            
            for k = 1:N+1
                opti.subject_to((obj.p(:,k) - obj.pL(:,k))' * H * (obj.p(:,k) - obj.pL(:,k)) >= 1);  %apparently this needs to be in a for loop `?
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

            try
                obj.sol = obj.opti.solve();   % Attempt to solve
            catch err
                disp('Solver failed!');
                disp(err.message);
                return;
            end
            disp("solve success in nmpcControl.solve")
            
            % Set warm start for next solve
            obj.opti.set_initial(obj.sol.value_variables());
            obj.opti.set_initial(obj.opti.lam_g, obj.sol.value(obj.opti.lam_g));
        end
    end
end
