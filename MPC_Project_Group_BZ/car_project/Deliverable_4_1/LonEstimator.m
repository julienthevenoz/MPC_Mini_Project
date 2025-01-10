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

            xs = sys.UserData.xs; 
            us = sys.UserData.us;
            
            % Discretize the system and extract the A,B,C,D matrices
            [~, Ad, Bd, Cd, ~] = Car.c2d_with_offset(sys, Ts);
            est.xs_hat = []; %[Vs_hat] ?
            est.us_hat = [];
            est.A_hat = [];
            est.B_hat = [];
            est.C_hat = []; 
            
            est.L = [];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            est.A_hat = [Ad(2,2) Bd(2);
                         0          1];
            est.B_hat = [Bd(2); 0];   
            est.C_hat = [Cd(1) 0];  
            poles = [0.5, 0.6];   %in the unit circle
            est.L = -place(est.A_hat', est.C_hat', poles')';  

            est.xs_hat = xs;
            est.us_hat = us; 
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
            
            estimation_error = z_hat(1) - y; 
            z_hat_next = est.A_hat*z_hat + est.B_hat*u + est.L * estimation_error;

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
