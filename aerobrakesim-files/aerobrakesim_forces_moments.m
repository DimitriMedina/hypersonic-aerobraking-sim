% aerobrakesim_forces_moments.m
%
% Generation of forces for sim
%
% Inputs:
%   States
%   Alpha
%   Time
%
% Outputs:
%   Forces
%
function out = aerobrakesim_forces_moments(uu, P)

    % Extract variables from input vector uu
    %   uu = [x(1:3); alpha; time(1)];
    k=(1:3);          x=uu(k);          % states
    k=k(end)+(1);     alpha=uu(k);      % Angle of Attack
    k=k(end)+(1);     time=uu(k);       % Simulation time, s

    % Extract state variables from x
    r    = x(1);   % altitude, km
    v    = x(2);   % speed, km/s
    gamma    = x(3);   % flight path angle, rad

    if(time==0)
        r = P.r0;
        v = P.v0;
        gamma = P.gamma0;
        alpha = P.uv(1);
    end

    rho = P.rho0*exp(-P.beta*r); %air density
    g = P.mu/((r+P.Re)^2); %gravity

    % Longitudinal Aero Coefficients
    C_L = P.C_L_alpha.*alpha; %coefficient of lift
    C_D = P.C_D_0 + P.K*C_L^2; %coefficient of Drag

    % Create and combine Forces
    D = 0.5*rho*v^2*P.S*C_D; %Drag
    L = 0.5*rho*v^2*P.S*C_L; %Lift

    % Compile function output
    out = [D, L]; % Length 2
end