% aerobrakesim_trajectory_control.m
%
% Pull the optimal control formula
%
% Inputs:
%   States
%   Time
%   Parameters
%
% Outputs:
%   a length-4 vector of [alpha_cmd, r_ref, v_ref, gamma_ref]
%
%   
function out = aerobrakesim_trajectory_control(uu, P)
    % Extract variables from input vector uu
    %   uu = [x(1:3); time(1)];
    k=(1:3);       estimates=uu(k); % Feedback state estimates
    k=k(end)+(1);   t=uu(k);      % Simulation time, s

    % Extract variables from estimates
    r           = estimates(1);  % Altitude [km]
    v           = estimates(2);  % Velocity [km/s]
    gamma       = estimates(3);  % flight path angle [rad]

    % Extract variables from optimal solution  
    r_opt       = P.xv(:,1); % Optimal Altitude [km]
    v_opt       = P.xv(:,2); % Optimal Velocity [km/s]
    gamma_opt   = P.xv(:,3); % Optimal flight path angle [rad]

    %ensure simulation time does not exceed our trajectory data limits
    t_safe = min(max(t, P.tv(1)), P.tv(end));
    
    % Feedforward Angle of Attack command
    alpha_cmd = interp1(P.tv,P.uv,t_safe,'linear','extrap');

    % Optimal States (PID Setpoint)
    r_ref = interp1(P.tv,r_opt,t_safe,'linear','extrap');
    v_ref = interp1(P.tv,v_opt,t_safe,'linear','extrap');
    gamma_ref = interp1(P.tv,gamma_opt,t_safe,'linear','extrap');

    % Compile output vector
    out=[alpha_cmd, r_ref, v_ref, gamma_ref];
end