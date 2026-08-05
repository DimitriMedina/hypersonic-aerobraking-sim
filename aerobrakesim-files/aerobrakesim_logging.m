% aerobrakesim_logging.m
%
% Logging of aerobrakesim variables
%
% Inputs:
%   Various 
%
% Outputs:
%   Creates a "flight_log" structure in the Matlab workspace
%
function aerobrakesim_logging(uu,P)
    % Extract variables from input vector uu
    %   uu = [x(1:3); estimates(1:3); meas(1:3); optimal(1:4); alpha_cmd(1); time(1)];
    k=1:3;           x=uu(k);         % States
    k=k(end)+(1:3);  estimates=uu(k); % Autopilot state estimates
    k=k(end)+(1:3);  meas=uu(k);      % Measurements
    k=k(end)+(1);    alpha_opti=uu(k); % Optimal alpha
    k=k(end)+(1:3);  optimal=uu(k);   % Optimal Trajectory
    k=k(end)+(1);    alpha_cmd=uu(k); % alpha cmd
    k=k(end)+(1);    time=uu(k);      % Simulation time, s

    % Extract state variables from true states
    r_true        = x(1);   % True altitude [km]
    v_true        = x(2);   % True velocity [km/s]
    gamma_true    = x(3);   % True flight path angle [rad]

    % Extract state variables from estimates (kalman filter)
    r_est         = estimates(1);   % Estimate altitude [km]
    v_est         = estimates(2);   % Estimate velocity [km/s]
    gamma_est     = estimates(3);   % Estimate flight path angle [rad]

    % Extract state variables from measurements
    r_meas        = meas(1);   % Measurement altitude [km]
    v_meas        = meas(2);   % Measurement velocity [km/s]
    gamma_meas    = meas(3);   % Measurement flight path angle [rad]
    
    % Extract state variables from optimal solution
    r_opti        = optimal(1);   % Optimal altitude [km]
    v_opti        = optimal(2);   % Optimal velocity [km/s]
    gamma_opti    = optimal(3);   % Optimal flight path angle [rad]

    % Use a persistent variable for incrementing logging index
    % Re-initialize "flight_log" structure if first time through
    persistent i
    if time==0
        i=0;
        flight_log = [];
        assignin('base','flight_log',flight_log);
    end
    i=i+1;

    % Acquire flight_log structure from base workspace
    flight_log = evalin('base','flight_log');

    % Append new data to flight_log structure
    flight_log.time_s(i) = time;

    % True States
    flight_log.r_true(i) = r_true;
    flight_log.v_true(i) = v_true;
    flight_log.gamma_true_deg(i) = rad2deg(gamma_true);

    % Extract a/p commands
    flight_log.alpha_cmd_deg(i) = rad2deg(alpha_cmd);
    
    % Extract sensor measurements
    flight_log.r_meas(i)      = r_meas; 
    flight_log.v_meas(i)      = v_meas;
    flight_log.gamma_meas_deg(i)  = rad2deg(gamma_meas);

    % Extract state estimates
    flight_log.r_est(i)      = r_est; 
    flight_log.v_est(i)      = v_est;
    flight_log.gamma_est_deg(i)  = rad2deg(gamma_est);

    % Extract Optimal Solution
    flight_log.r_opti(i)      = r_opti; 
    flight_log.v_opti(i)      = v_opti;
    flight_log.gamma_opti_deg(i) = rad2deg(gamma_opti);
    flight_log.alpha_opti_deg(i) = rad2deg(alpha_opti);

    % Write flight_log structure back to workspace
    assignin('base','flight_log',flight_log);
end