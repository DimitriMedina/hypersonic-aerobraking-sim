clc; clear all;close all;format compact;

[problem,guess]=FinalProject;          % Fetch the problem definition
options= problem.settings(50);                  % Get options and solver settings 
[solution,MRHistory]=solveMyProblem( problem,guess,options);
[ tv, xv, uv ] = simulateSolution( problem, solution, 'ode113', 0.1 );

%% figure
xx=linspace(solution.T(1,1),solution.tf,100000);

figure
hold on
plot(xx,(speval(solution,'X',1,xx)),'b-' )
xlim([0 solution.tf])
plot(tv,(xv(:,1)),'k-.')
xlabel('Time [s]')
ylabel('h [m]')
title('Altitude')
grid on

figure
hold on
plot(xx,speval(solution,'X',2,xx),'b-' )
xlim([0 solution.tf])
plot(tv,xv(:,2),'k-.')
xlabel('Time [s]')
ylabel('Velocity [km/s]')
title('Velocity')
grid on

figure
hold on
plot(xx,speval(solution,'X',3,xx)*180/pi,'b-' )
xlim([0 solution.tf])
plot(tv,xv(:,3)*180/pi,'k-.')
xlabel('Time [s]')
ylabel('Flight Path Angle [deg]')
title('Flight Path Angle')
grid on


%%
figure
hold on
plot(xx,speval(solution,'U',1,xx)*180/pi,'b-' )
xlim([0 solution.tf])
plot(tv,uv(:,1)*180/pi,'k-.')
xlabel('Time [s]')
ylabel('Angle of Attack [deg]')
title('Angle of Attack')
grid on
