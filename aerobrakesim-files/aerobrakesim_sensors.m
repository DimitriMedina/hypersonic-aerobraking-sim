% aerobrakesim_sensors.m
%
% Generation of sensor measurements for aerobrakesim
%
% Inputs:
%   Forces
%   States
%   Time
%
% Outputs:
%   Sensor Measurements
%
function out = aerobrakesim_sensors(uu, P)

    % Extract variables from input vector uu
    %   uu = [forces(1:2); x(1:3); time(1)];
    k=(1:2);           forces=uu(k);    % Forces (Drag and Lift)
    k=k(end)+(1:3);    x=uu(k);         % states 
    k=k(end)+(1);      time=uu(k);      % Simulation time, s

    % Extract forces and moments from f_and_m
    D = forces(1); % Drag N-km
    L = forces(2); % Lift N-km

    % Extract state variables from x
    r        = x(1);   % Altitude [km]
    v        = x(2);   % Velocity [km/s]
    gamma    = x(3);   % Flight Path Angle [rad]

    if(time==0)
        r = P.r0;
        v = P.v0;
        gamma = P.gamma0;
        alpha = P.uv(1);
    end

    % GPS Position and Velocity Measurements
    persistent time_gps_prev ...
                gps_alt_error alt_gps v_gps
    if(time==0)
        gps_alt_error = P.sigma_bias_gps_alt*randn;
        time_gps_prev = -inf; % Force update at time==0
    end

    if(time>time_gps_prev+P.Ts_gps)
        % Gauss-Markov growth of GPS position errors
        gps_alt_error   = exp(-P.Ts_gps/P.tau_gps)*gps_alt_error + P.sigma_eta_gps_alt*randn*sqrt(P.Ts_gps);

        % GPS Position Measurements
        alt_gps = r + gps_alt_error;

        % GPS Velocity Measurements
        v_gps = v + P.sigma_noise_gps_speed*randn;

        time_gps_prev = time;
    end

    % IMU Measurement
    gamma_imu = gamma + P.sigma_noise_imu*rand;

    % Compile output vector
    out = [ ...
            alt_gps;...
            v_gps; ...
            gamma_imu
          ]; % Length: 18

end
