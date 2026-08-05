function [problem,guess] = FinalProject
%FinalProject
%
% Syntax:  [problem,guess] = FinalProject
%
% Outputs:
%    problem - Structure with information on the optimal control problem
%    guess   - Guess for state, control and multipliers.
%
% Other m-files required: none
% MAT-files required: none

%------------- BEGIN CODE --------------

% Earth Conditions
Re = 6378.400; %km
mu = 398970; %km^3/s^2
hatm = 129.60; %km
C_L_max = 1;

% Boundary Conditions
r_0 = hatm; %km
v_0 = 8.50032; %km/s
gamma_0 = -4.5*pi/180; %rad

r_f = hatm; %km
v_f = 8.20056; %km/s
gamma_f = 2*pi/180;
rate_const = deg2rad(5);
speed_const = 100;
max_pitch_rate = 5*pi/180;

% variable simple bounds
r_min = 0; r_max = hatm*1.1;
v_min = 1; v_max = v_0*1.1;
gamma_min = -89*pi/180; gamma_max = 89*pi/180;
alpha_min = -90*pi/180; alpha_max = 90*pi/180;

%%

% Plant model name, used for Adigator
InternalDynamics=@FinalProject_Dynamics_Internal;
SimDynamics=@FinalProject_Dynamics_Sim;

% Analytic derivative files (optional)
problem.analyticDeriv.gradCost=[];
problem.analyticDeriv.hessianLagrangian=[];
problem.analyticDeriv.jacConst=[];

% Settings file
problem.settings=@settings_FinalProject;

%Initial Time. t0<tf
problem.time.t0_min=0;
problem.time.t0_max=0;
guess.t0=0;

% Final time. Let tf_min=tf_max if tf is fixed.
problem.time.tf_min=0;     
problem.time.tf_max=40000; 
guess.tf=2000;

% Parameters bounds. pl=< p <=pu
problem.parameters.pl=[0];
problem.parameters.pu=[6000000]; %MW/km^2
guess.parameters=[4000000]; %MW/km^2

% Initial conditions for system.
problem.states.x0 = [r_0 v_0 gamma_0];

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
problem.states.x0l = [r_0 v_0 gamma_0]; 
problem.states.x0u = [r_0 v_0 gamma_0]; 

% State bounds. xl=< x <=xu
problem.states.xl=[r_min v_min gamma_min];
problem.states.xu=[r_max v_max gamma_max];

% State rate bounds. xrl=< x_dot <=xru
problem.states.xrl=[-inf -inf -inf];
problem.states.xru=[inf inf inf];

% State error bounds
problem.states.xErrorTol_local=[1 0.1 deg2rad(0.5)];
problem.states.xErrorTol_integral=[10  0.1 deg2rad(0.5)];

% State constraint error bounds
problem.states.xConstraintTol=[1 0.1 deg2rad(0.5)];
problem.states.xrConstraintTol=[1 0.1 deg2rad(0.5)];

% Terminal state bounds. xfl=< xf <=xfu
problem.states.xfl=[r_f v_f-0.01 0]; 
problem.states.xfu=[r_f v_f+0.01 inf];

% Guess the state trajectories with [x0 xf]
guess.time = [0 guess.tf/2, guess.tf];
guess.states(:,1)=[r_0 60 r_f];
guess.states(:,2)=[v_0 8.35 v_f];
guess.states(:,3)=[gamma_0 0 gamma_f];

% Number of control actions N 
% Set problem.inputs.N=0 if N is equal to the number of integration steps.  
% Note that the number of integration steps defined in settings.m has to be divisible 
% by the  number of control actions N whenever it is not zero.
problem.inputs.N=0;       
      
% Input bounds
problem.inputs.ul=[alpha_min];
problem.inputs.uu=[alpha_max];

problem.inputs.u0l=[alpha_min];
problem.inputs.u0u=[alpha_max];

% Input rate bounds
problem.inputs.url=[-inf]; 
problem.inputs.uru=[inf]; 

% Input constraint error bounds
problem.inputs.uConstraintTol=[deg2rad(0.5)];
problem.inputs.urConstraintTol=[deg2rad(0.5)];

% Guess the input sequences with [u0 uf]
guess.inputs(:,1)=[10 20 10]*pi/180;

% Choose the set-points if required
problem.setpoints.states=[];
problem.setpoints.inputs=[];

% Bounds for path constraint function gl =< g(x,u,p,t) =< gu
problem.constraints.ng_eq=0;
problem.constraints.gTol_eq=[];

problem.constraints.gl=[0, -C_L_max];
problem.constraints.gu=[inf, C_L_max];
problem.constraints.gTol_neq=[0.1, 1e-03];

% Bounds for boundary constraints bl =< b(x0,xf,u0,uf,p,t0,tf) =< bu
problem.constraints.bl=[];
problem.constraints.bu=[];
problem.constraints.bTol=[];


% store the necessary problem parameters used in the functions
problem.data.mu = mu;
problem.data.mass = 4898.7; %kg
problem.data.Re = Re;
problem.data.S  = 11.69/1e6; %km^2
problem.data.Isp = 310; %s
problem.data.beta = 1/7.200; %1/km
problem.data.rho0 = 1.225*1e9; %kg/km^3
problem.data.Qdot = 1.9987*1e+8; %MW/km^2
problem.data.K = 1.4;
problem.data.C_D_0 = 0.032;
problem.data.C_L_alpha = 0.5699;
problem.data.C_L_max = C_L_max;

% Get function handles and return to Main.m
problem.data.InternalDynamics=InternalDynamics;
problem.data.functionfg=@fg;
problem.data.plantmodel = func2str(InternalDynamics);
problem.functions={@L,@E,@f,@g,@avrc,@b};
problem.sim.functions=SimDynamics;
problem.sim.inputX=[];
problem.sim.inputU=1:length(problem.inputs.ul);
problem.functions_unscaled={@L_unscaled,@E_unscaled,@f_unscaled,@g_unscaled,@avrc,@b_unscaled};
problem.data.functions_unscaled=problem.functions_unscaled;
problem.data.ng_eq=problem.constraints.ng_eq;
problem.constraintErrorTol=[problem.constraints.gTol_eq,problem.constraints.gTol_neq,problem.constraints.gTol_eq,problem.constraints.gTol_neq,problem.states.xConstraintTol,problem.states.xConstraintTol,problem.inputs.uConstraintTol,problem.inputs.uConstraintTol];

%------------- END OF CODE --------------

function stageCost=L_unscaled(x,xr,u,ur,p,t,vdat)

% L_unscaled - Returns the stage cost.
% The function must be vectorized and
% xi, ui are column vectors taken as x(:,i) and u(:,i) (i denotes the i-th
% variable)
% 
% Syntax:  stageCost = L(x,xr,u,ur,p,t,data)
%
% Inputs:
%    x  - state vector
%    xr - state reference
%    u  - input
%    ur - input reference
%    p  - parameter
%    t  - time
%    data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    stageCost - Scalar or vectorized stage cost
%
%  Remark: If the stagecost does not depend on variables it is necessary to multiply
%          the assigned value by t in order to have right vector dimesion when called for the optimization. 
%          Example: stageCost = 0*t;

%------------- BEGIN CODE --------------


stageCost = 0*t;

%------------- END OF CODE --------------


function boundaryCost=E_unscaled(x0,xf,u0,uf,p,t0,tf,data) 

% E_unscaled - Returns the boundary value cost
%
% Syntax:  boundaryCost=E_unscaled(x0,xf,u0,uf,p,t0,tf,data) 
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    boundaryCost - Scalar boundary cost
%
%------------- BEGIN CODE --------------

boundaryCost=p(1);

%------------- END OF CODE --------------


function bc=b_unscaled(x0,xf,u0,uf,p,t0,tf,vdat,varargin)

% b_unscaled - Returns a column vector containing the evaluation of the boundary constraints: bl =< bf(x0,xf,u0,uf,p,t0,tf) =< bu
%
% Syntax:  bc=b_unscaled(x0,xf,u0,uf,p,t0,tf,vdat,varargin)
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
%
%          
% Output:
%    bc - column vector containing the evaluation of the boundary function 
%
%------------- BEGIN CODE --------------
varargin=varargin{1};
bc=[];
%------------- END OF CODE --------------
% When adpative time interval add constraint on time
%------------- BEGIN CODE --------------
if length(varargin)==2
    options=varargin{1};
    t_segment=varargin{2};
    if ((strcmp(options.discretization,'hpLGR')) || (strcmp(options.discretization,'globalLGR')))  && options.adaptseg==1 
        if size(t_segment,1)>size(t_segment,2)
            bc=[bc;diff(t_segment)];
        else
            bc=[bc,diff(t_segment)];
        end
    end
end

%------------- END OF CODE --------------

