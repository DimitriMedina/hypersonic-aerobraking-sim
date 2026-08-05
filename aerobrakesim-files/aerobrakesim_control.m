% aerobrakesim_control.m
%
% Flight control logic for uavsim
%
% Inputs:
%   Trajectory commands
%   State Feedbacks
%   Time
%
% Outputs:
%   alpha commands
%   
function out = aerobrakesim_control(uu,P)
    % Extract variables from input vector uu
    %   uu = [optimal_state(1:3); estimates(1:3); traj_cmds(1); time(1)];
    k=1;                alpha_cmd=uu(k);   % Optimal angle of attack
    k=k(end)+(1:3);     states_opt=uu(k);  % Optimal state
    k=k(end)+(1:3);     estimates=uu(k);   % Feedback state estimates
    k=k(end)+(1);       t=uu(k);        % Simulation time, s

    % Extract variables from estimates
    r       = estimates(1);  % estimated position [km]
    v       = estimates(2);  % estimated speed [km/s]
    gamma   = estimates(3);  % estimated flight path angle [rad]

    % Extract variables from estimates
    r_ref       = states_opt(1);  % optimal position [km]
    v_ref      = states_opt(2);  % optimal speed [km/s]
    gamma_ref   = states_opt(3);  % optimal flight path angle [rad]

    persistent int_r gamma_int prev_r prev_gamma prev_t
    if isempty(int_r), [int_r, gamma_int, prev_r, prev_gamma, prev_t] = deal(0); end
    dt = t - prev_t;

    Kp_alt = 0.01;

    Kp = 1;
    Ki = 0.1;
    Kd = 0.5;

    if dt > 0
        % Outer loop
        err_r = r_ref - r;
        gamma_correction = Kp_alt*err_r;

        % Inner Loop
        gamma_target = gamma_ref + gamma_correction;
        err_gamma = gamma_target - gamma;

        % PID for Gamma
        gamma_int = gamma_int+ (err_gamma*dt);
        gamma_int = max(min(gamma_int, 0.5), -0.5);

        gamma_d = (err_gamma - prev_gamma) / dt;
        alpha_fb = (Kp*err_gamma) + (Ki*gamma_int) + (Kd*gamma_d);

        prev_r = err_r;
        prev_gamma = err_gamma;
        prev_t = t;
    else
        alpha_fb = 0;
    end

    % Combine optimal Feedforward with PID Feedback
    alpha_c = alpha_cmd + alpha_fb;

    % Alpha limits
    alpha_limit = deg2rad(45);
    alpha_c = max(min(alpha_c, alpha_limit), -alpha_limit);

    % Compile autopilot commands for logging/vis
    ap_command = [ ...
            alpha_c;...
        ];

    % Compile output vector
    out=[alpha_c];
end