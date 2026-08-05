% aerobrakesim_kin_dyn.m
%
% Kinematics and Dynamics for sim
%
% Inputs:
%   States
%   Forces
%   Time
%
% Outputs:
%   Vector of state derivatives
%   
function out = aerobrakesim_kin_dyn(uu,P)

    % Extract variables from input vector uu
    %   uu = [x(1:3); forces(1:2); time(1)];
    k=1:3;           x=uu(k);         % States
    k=k(end)+(1:2);  forces=uu(k);    % Forces
    k=k(end)+(1);    time=uu(k);      % Simulation time, s

    % Extract state variables from x
    r    = x(1);   % altitude, km
    v    = x(2);   % speed, km/s
    gamma    = x(3);   % flight path angle, rad

    % Extract body-frame forces and moments
    D   = forces(1); % Drag Force N-km
    L   = forces(2); % Lift N-km

    g = P.mu/((r+P.Re).^2);

    rdot = v.*sin(gamma);
    vdot = -D/P.mass - g.*sin(gamma);
    gammadot = L/P.mass./v + cos(gamma).*(v/(r + P.Re) - g/v);

    % Compile state derivatives vector
    xdot = [rdot, vdot, gammadot];

    % Compile function ouput
    out = xdot;
end