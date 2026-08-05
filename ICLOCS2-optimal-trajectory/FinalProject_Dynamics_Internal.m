function [dx,g_neq] = FinalProject_Dynamics_Internal(x,u,p,t,data)
%FinalProject - Dynamics - Internal
%
% Syntax:  
%          [dx] = Dynamics(x,u,p,t,vdat)	(Dynamics Only)
%          [dx,g_eq] = Dynamics(x,u,p,t,vdat)   (Dynamics and Eqaulity Path Constraints)
%          [dx,g_neq] = Dynamics(x,u,p,t,vdat)   (Dynamics and Inqaulity Path Constraints)
%          [dx,g_eq,g_neq] = Dynamics(x,u,p,t,vdat)   (Dynamics, Equality and Ineqaulity Path Constraints)
% 
% Inputs:
%    x  - state vector
%    u  - input
%    p  - parameter
%    t  - time
%    vdat - structured variable containing the values of additional data used inside
%          the function%      
% Output:
%    dx - time derivative of x
%    g_eq - constraint function for equality constraints
%    g_neq - constraint function for inequality constraints
%
%
%------------- BEGIN CODE --------------


r = x(:,1);
v = x(:,2);
gamma = x(:,3);
alpha = u(:,1);
qU=p(:,1);

rho = data.rho0.*exp(-data.beta.*r);
C_L = data.C_L_alpha.*alpha;
C_D = data.C_D_0 + data.K*C_L.^2;
D = 0.5*rho.*v.^2.*data.S.*C_D;
L = 0.5*rho.*v.^2.*data.S.*C_L;
g = data.mu./((r+data.Re).^2);

dr = v.*sin(gamma);
dv = -D./data.mass-g.*sin(gamma);
dgamma = L./data.mass./v+cos(gamma).*(v./(r + data.Re)-g./v);

dx = [dr, dv, dgamma];

qdot = data.Qdot.*(rho./data.rho0).^0.5.*(v./sqrt(data.mu./data.Re)).^3.15;

g_neq = [qU-qdot, C_L];
