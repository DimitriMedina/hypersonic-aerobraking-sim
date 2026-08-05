% aerobrakesim_estimates.m
%
% Generation of feedback state estimates for uavsim
%
% Inputs:
%   Measurements
%   Time
%
% Outputs:
%   Feedback state estimates
%   
function out = aerobrakesim_estimates(uu,P)
    % Extract variables from input vector uu
    %   uu = [meas(1:3); time(1)];
    k=(1:3);               meas=uu(k);   % Sensor Measurements
    k=k(end)+(1);          alpha = uu(k); % angle of attack [rad]
    k=k(end)+(1);          time=uu(k);   % Simulation time, s

    % Extract mesurements
    k=1;
    alt_gps     = meas(k);    k=k+1; % Altitude [km]
    v_gps       = meas(k);    k=k+1; % Velocity [km/s]
    gamma_meas  = meas(k);    k=k+1; % Flight Path Angle [rad]

    % Filter raw measurements
    persistent lpf_gamma
    if(time==0)
        % Filter initializations
        lpf_gamma = gamma_meas;
    end
    % NOTE: You need to modify LPF(), see below end of this file.
    lpf_gamma = LPF(gamma_meas,lpf_gamma,P.tau_imu,P.Ts);

    %% EKF to estimate position, velocity, flight path angle
    dt = P.Ts;
    Q = diag([1e-10, 1e-7]); % Continuous-time process noise matrix
    R = diag([P.sigma_eta_gps_alt^2,...
              (P.sigma_noise_gps_speed^2)]);
    persistent xhat P_hat
    if(time==0)
        xhat=[P.r0;
              P.v0]; % States: [r; v; gamma]
        P_hat=diag([1^2, 0.1^2]);
    end

    N=10; % Number of sub-steps for propagation each sample period
    for i=1:N % Prediction step (N sub-steps)
        r = xhat(1);
        v = xhat(2);

        rho = P.rho0.*exp(-P.beta.*r);
        C_L = P.C_L_alpha.*alpha;
        C_D = P.C_D_0 + P.K*C_L.^2;
        Drag = 0.5*rho.*v.^2.*P.S.*C_D;
        Lift = 0.5*rho.*v.^2.*P.S.*C_L;

        f_x = [v*sin(lpf_gamma);
               -Drag./P.mass - P.mu/(r+P.Re)^2*sin(lpf_gamma)]; % State derivatives, xdot = f(x,...)
        A = [0, sin(lpf_gamma);
             2*P.mu*sin(lpf_gamma)/(r+P.Re)^3, -rho*v*P.S*C_D/P.mass]; % Linearization (Jacobian) of f(x,...) wrt x
        
        xhat = xhat + (dt/N)*f_x; % States propagated to sub-step N
        P_hat = P_hat + (dt/N)*(A*P_hat + P_hat*A' + Q); % Covariance matrix propagated to sub-step N
        P_hat = real(.5*P_hat + .5*P_hat'); % Make sure P stays real and symmetric
    end
    
    y_meas = [alt_gps; v_gps]; % Vector of actual measurements
    
    % Measurement Model and Jacobian
    h = xhat;
    C = eye(2);

    % Calculate Kalman Gain
    L             = P_hat*(C')*(C*P_hat*(C') + R)^-1; % Kalman Gain matrix
    P_hat         = (eye(2) - L*C)*P_hat; % Covariance matrix updated with measurement information
    xhat          = xhat + L*(y_meas - h); % States updated with measurement information 
    r_hat_unc     = sqrt(P_hat(1,1)); % EKF-predicted uncertainty in altitude estimate, km 
    v_hat_unc     = sqrt(P_hat(2,2)); % EKF-predicted uncertainty in speed estimate, km/s

    % estimate states
    r_hat         = xhat(1);
    v_hat         = xhat(2);
    gamma_hat     = lpf_gamma;
    
    % Compile output vector
    out = [...
            r_hat;...         % 1
            v_hat;...         % 2
            gamma_hat;...     % 3
          ]; % Length: 3
    
end 


function y = LPF(u,yPrev,tau,Ts)
%
%  Y(s)       a           1
% ------ = ------- = -----------,  tau: Filter time contsant, s
%  U(s)     s + a     tau*s + 1         ( tau = 1/a )
%
    alpha_LPF = exp(-Ts/tau);
    y = alpha_LPF*yPrev + (1 - alpha_LPF)*u;
end
