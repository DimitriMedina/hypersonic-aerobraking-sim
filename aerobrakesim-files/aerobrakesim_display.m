% aerobrakesim_display.m
%
% Visualization of uavsim variables
%
% Inputs:
%   Various 
%
% Outputs:
% 
%
% Extract variables from input vector uu

close all
% first time function is called, initialize plot and persistent vars
figure(1)
plot(flight_log.time_s,flight_log.r_true,'r--'); hold on
plot(flight_log.time_s,flight_log.r_est,'color',[0 .5 0]); hold on
plot(flight_log.time_s,flight_log.r_meas,'b-'); hold on
grid on;
xlabel('Time [s]');
ylabel('Alt [km]');
legend('True','Estimate','Measured');

figure(2)
plot(flight_log.time_s,flight_log.v_true,'r--'); hold on
plot(flight_log.time_s,flight_log.v_est,'color',[0 .5 0]); hold on
plot(flight_log.time_s,flight_log.v_meas,'b-'); hold on
grid on;
xlabel('Time [s]');
ylabel('Velocity [km/s]');
legend('True','Estimate','Measured');

figure(3)
plot(flight_log.time_s,flight_log.gamma_true_deg,'r--'); hold on
plot(flight_log.time_s,flight_log.gamma_est_deg,'color',[0 .5 0]); hold on
plot(flight_log.time_s,flight_log.gamma_meas_deg,'b-'); hold on
grid on;
xlabel('Time [s]');
ylabel('flight path angle [deg]');
legend('True','Estimate','Measured');

figure(4)
plot(flight_log.time_s,flight_log.r_true,'r--'); hold on
plot(flight_log.time_s,flight_log.r_opti,'b-'); hold on
grid on;
xlabel('Time [s]');
ylabel('Alt [km]');
title('True vs. Reference Alt');
legend('True','Optimal');

figure(5)
plot(flight_log.time_s,flight_log.v_true,'r--'); hold on
plot(flight_log.time_s,flight_log.v_opti,'b-'); hold on
grid on;
xlabel('Time [s]');
ylabel('Velocity [km/s]');
title('True vs. Reference Velocity');
legend('True','Optimal');

figure(6)
plot(flight_log.time_s,flight_log.gamma_true_deg,'r--'); hold on
plot(flight_log.time_s,flight_log.gamma_opti_deg,'b-'); hold on
grid on;
xlabel('Time [s]');
ylabel('flight path angle [deg]');
title('True vs. Reference Flight Path Angle');
legend('True','Optimal');

figure(7)
plot(flight_log.time_s,flight_log.r_opti - flight_log.r_true,'r--'); hold on
grid on;
xlabel('Time [s]');
ylabel('Alt [km]');
title('Tracking Error Alt');
legend('Error');

figure(8)
plot(flight_log.time_s,flight_log.v_opti - flight_log.v_true,'r--'); hold on
grid on;
xlabel('Time [s]');
ylabel('Velocity [km/s]');
title('Tracking Error Velocity');    
legend('Error');

figure(9)
plot(flight_log.time_s,flight_log.gamma_opti_deg - flight_log.gamma_true_deg,'r--'); hold on
grid on;
xlabel('Time [s]');
ylabel('flight path angle [deg]');
title('Tracking Error Flight Path Angle'); 
legend('Error');

figure(10)
plot(flight_log.time_s,flight_log.alpha_cmd_deg,'r--'); hold on
grid on;
xlabel('Time [s]');
ylabel('Angle of Attack [deg]')
title('Tracked Flight Command Alpha');
legend('Applied Command');

figure(11)
plot(flight_log.time_s,flight_log.alpha_cmd_deg,'r--'); hold on
plot(flight_log.time_s,flight_log.alpha_opti_deg,'b-'); hold on
grid on;
xlabel('Time [s]');
ylabel('Angle of Attack [deg]')
title('Optimal vs Applied Flight Command Alpha');
legend('Applied Command','Optimal Command');

figure(12)
plot(flight_log.time_s,flight_log.alpha_opti_deg- flight_log.alpha_cmd_deg,'r--'); hold on
grid on;
xlabel('Time [s]');
ylabel('Angle of Attack [deg]')
title('Tracking Error Flight Command Alpha [deg]');   
legend('Error');

figure(13)
plot(flight_log.time_s,flight_log.r_est - flight_log.r_true,'r--'); hold on
grid on;
xlabel('Time [s]');
ylabel('Alt [km]');
title('Estimate Error Alt');
legend('Error');

figure(14)
plot(flight_log.time_s,flight_log.v_est - flight_log.v_true,'r--'); hold on
grid on;
xlabel('Time [s]');
ylabel('Velocity [km/s]');
title('Estimate Error Velocity');    
legend('Error');

figure(15)
plot(flight_log.time_s,flight_log.gamma_est_deg - flight_log.gamma_true_deg,'r--'); hold on
grid on;
xlabel('Time [s]');
ylabel('flight path angle [deg]');
title('Estimate Error Flight Path Angle'); 
legend('Error');