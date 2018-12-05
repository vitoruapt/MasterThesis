function LaneFollowingResults(logsout)
% Plot results for adaptive MPC. 

%% Get the data from simulation

[e1_ampc,e2_ampc,delta_ampc,accel_ampc,vx_ampc,vy_ampc] = getData(logsout);

%% Plot results. 
figure; % lateral results
% steering angle
subplot(3,1,1);
hold on;
grid on;
grid minor;
plot(delta_ampc.Values.time,delta_ampc.Values.Data);
title('\textbf{Steering Angle (u$_2$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Steering angle [rad]','Interpreter','latex');
hold off;
% lateral deviation
subplot(3,1,2);
hold on;
grid on;
grid minor;
plot(e1_ampc.Values.time,e1_ampc.Values.Data);
title('\textbf{Lateral Deviation (e$_1$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Lateral Deviation [m]','Interpreter','latex');
hold off;
% relative yaw angle
subplot(3,1,3);
hold on;
grid on;
grid minor;
plot(e2_ampc.Values.Time,e2_ampc.Values.Data);
title('\textbf{Relative Yaw Angle (e$_2$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Relative Yaw Angle [rad]','Interpreter','latex');
hold off;

figure; % longitudinal results
% acceleration
subplot(2,1,1);
hold on;
grid on;
grid minor;
plot(accel_ampc.Values.time,accel_ampc.Values.Data);
title('\textbf{Acceleration (u$_1$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Acceleration [m/s$^2$]','Interpreter','latex');
hold off;
% longitudinal velocity
subplot(2,1,2);
hold on;
grid on;
grid minor;
plot(vx_ampc.Values.Time,vx_ampc.Values.Data);
title('\textbf{Velocity (V$_y$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Velocity [m/s]','Interpreter','latex');
hold off;

%% Local function: Get data from simulation
function [e1,e2,delta,accel,vx,vy,xp,yp] = getData(logsout)
e1 = logsout.getElement('Lateral Deviation');    % lateral deviation
e2 = logsout.getElement('Relative Yaw Angle');   % relative yaw angle
delta = logsout.getElement('Steering');          % steering angle
accel = logsout.getElement('Acceleration');      % acceleration of ego car
vx = logsout.getElement('Longitudinal Velocity');% velocity of host car
vy = logsout.getElement('Lateral Velocity');