classdef LonEstimator
    properties
        % continous sub-system
        sys
        % Extended linearization points
        xs_hat, us_hat
        % Extended system matrices
        A_hat, B_hat, C_hat
        % Observer gain matrix
        L
    end
    
    methods
        % Setup the longitudinal disturbance estimator
        function est = LonEstimator(sys, Ts)

            xs = sys.UserData.xs; %contains only [Vs] ?
            us = sys.UserData.us;
            
            % Discretize the system and extract the A,B,C,D matrices
            [~, Ad, Bd, Cd, Dd] = Car.c2d_with_offset(sys, Ts);
  
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            %we only care about measuring V and disturbance d
            % let's define the vector estimation = [V_hat, d_hat] and error = predicted output - measured output = C*state + Cd*d - measurement
            %we have next_estimation = A_hat*estimation + B_hat*u_s + L_hat*error
            % Extended state dynamics

            %since we're only interested by V and d, we're only interested by the parts of the discretized matrices A and B which affect V,
            % ie bottom component of B and the bottom row of A (but actually bottom left is 0, so we only want the bottom right) 
            % -> we need A(2,2) and B(2)
            %Cd is the discretized version of observation matrix C. In our case I think they are both identity ?
            est.A_hat = [Ad(2,2), Bd(2);
                         0          1];
            est.B_hat = [Bd(2); 0];   %2x1 vector
            %est.C_hat = [Cd zeros(ny,1)];
            est.C_hat = [Cd(1) 0];  %this is the observation matrix ?
            poles = [0.5, 0.6];
            est.L = -place(est.A_hat', est.C_hat', poles')';  %we transpose L to get a 2x1 column vector
            %eigs(est.A_hat + est.L * est.C_hat)


            % Extended linearization points
            % Solve for xs_hat and us_hat at steady state
            % At steady state: x_s = Ad*x_s + Bd*u_s
            %                  y_ref = Cd*x_s + Dd*u_s
            y_ref = xs;  % Assume the reference is the nominal state value
            
            % Formulate the steady-state equations
            eq_mat = [eye(size(Ad)) - Ad, -Bd;
                      Cd,              Dd ];
            rhs = [zeros(size(Ad, 1), 1);  % 0 for steady-state dynamics
                   y_ref];                 % Reference output
            
            % Solve the system of equations
            sol = eq_mat \ rhs;
            
            % Extract the steady-state states and inputs
            est.xs_hat = sol(1:size(Ad, 1));  % First part of solution is x_s
            est.us_hat = sol(size(Ad, 1) + 1:end);  % Second part is u_s
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        % This function takes in the the estimate, input, and measurement
        % at timestep i and predicts the next (extended) state estimate at
        % the next timestep i + 1.
        function z_hat_next = estimate(est, z_hat, u, y)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   z_hat      - current estimate (V, dist)
            %   u          - longitudinal input (u_T)
            %   y          - longitudinal measurement (x, V)  %actually it's just V idiot
            % OUTPUTS
            %   z_hat_next - next time step estimate
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % Estimation equation
            estimation_error = z_hat(1) - y; % predicted V - measured V is the error in our estiamtion
            z_hat_next = est.A_hat*z_hat + est.B_hat*u + est.L * estimation_error;

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
