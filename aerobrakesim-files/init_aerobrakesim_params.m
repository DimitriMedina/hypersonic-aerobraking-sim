% init_aerobrakesim_params.m
%
% Initialize parameters structure (P) for aerobrakesim
%
% Inputs:
%   None
%
% Outputs:
%   P: parameters structure used for uavsim
%   
function P = init_aerobrakesim_params

    P = [];

    P.Ts = 0.01;     % Autopilot sample time step, s
    P.Tlog = 0.01;   % Logging time step, s
    P.Tvis = 0.20;   % Visualization time step, s

    % Environment params
    P.rho0 = 1.225*1e9; % air density kg/km^3
    P.Re = 6378.400; %km
    P.mu = 398970;
    P.beta = 1/7.200; %1/km

    % Initial Conditions
    P.r0 = 129.60; %km
    P.v0 = 8.50032; %km/s
    P.gamma0 = -4.5*pi/180; %rad

    % Boundary Conditions
    P.gamma_min = -89*pi/180; P.gamma_max = 89*pi/180;
    P.alpha_min = -90*pi/180; P.alpha_max = 90*pi/180;

    % Vehicle Parameters
    P.mass = 4898.7; %kg
    P.S  = 11.69/1e6; %km^2
    P.Qdot = 1.9987*1e+8; %MW/km^2
    P.K = 1.4;
    P.C_D_0 = 0.032;
    P.C_L_alpha = 0.5699;
    P.C_L_max = 1;

    % Sensor parameters: IMU
    P.sigma_noise_imu = 0.1*pi/180; % rad
    P.tau_imu = 0.2; %rad 

    % Sensor parameters: GPS
    P.Ts_gps = 10; % GPS sampling time, s
    P.sigma_bias_gps_alt   = 9.2/100;  % GPS alt   Gauss-Markov bias parameter, km
    P.sigma_eta_gps_alt   = 0.40/100;  % GPS alt   Gauss-Markov noise parameter, km
    P.tau_gps = 1100;  % GPS Gauss-Markov time constant, s (In book: k_gps = 1/tau_gps)
    P.sigma_noise_gps_speed = 0.1/100; % GPS groundspeed std. dev., km/s

    % Optimal Control Solution
    [problem,guess]=FinalProject;          % Fetch the problem definition
    options= problem.settings(50);                  % Get options and solver settings 
    [solution,MRHistory]=solveMyProblem( problem,guess,options);
    [tv, xv, uv] = simulateSolution( problem, solution, 'ode113', 0.1 );
    
    P.tv = tv; %optimal control time s
    P.xv = xv; %optimal control states
    P.uv = uv; %optimal control alpha [rad]
end