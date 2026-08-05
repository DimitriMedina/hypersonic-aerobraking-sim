function dx = FinalProject_Dynamics_Sim(x,u,p,t,data)
%FinalProject - Dynamics - simulation
%
% Syntax:  
%          [dx] = Dynamics(x,u,p,t,vdat)
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
%------------- BEGIN CODE --------------

r = x(:,1);
v = x(:,2);
gamma = x(:,3);
alpha= u(:,1);
qU=p(:,1);

rho = data.rho0.*exp(-data.beta.*r);
C_L = data.C_L_alpha.*alpha;
C_D = data.C_D_0 + data.K*C_L.^2;
D = 0.5*rho.*v.^2.*data.S.*C_D;
L = 0.5*rho.*v.^2.*data.S.*C_L;
g = data.mu./((r+data.Re).^2);

dr = v.*sin(gamma);
dv = -D./data.mass-g.*sin(gamma);
dgamma = L./data.mass./v+cos(gamma).*(v./(r+data.Re)-g./v);

dx = [dr, dv, dgamma];