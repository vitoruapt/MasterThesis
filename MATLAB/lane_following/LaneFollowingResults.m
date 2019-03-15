function LaneFollowingResults(logsout)
% Plot results for adaptive MPC. 

%% Get the data from simulation

[e1_ampc,e2_ampc,delta_ampc,accel_ampc,vx_ampc,vy_ampc] = getData(logsout);

%% Steering Angle vs Time

h7 = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
plot(delta_ampc.Values.time,delta_ampc.Values.Data);
hold on;
grid on;
grid minor;
%title('\textbf{Steering Angle (u$_2$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Steering angle [rad]','Interpreter','latex');
pos7 = get(h7,'Position');
set(h7,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos7(3)*0.8, pos7(4)-40])
print(h7,'figure\SteeringAngleVsTime','-dpdf','-r0')
hold off
pause(2)
%% Lateral deviation vs Time

h8 = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
plot(e1_ampc.Values.time,e1_ampc.Values.Data);
hold on;
grid on;
grid minor;
%title('\textbf{Lateral Deviation (e$_1$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Lateral Deviation [m]','Interpreter','latex');
pos8 = get(h8,'Position');
set(h8,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos8(3)*0.8, pos8(4)-40])
print(h8,'figure\LateralDeviationVsTime','-dpdf','-r0')
hold off
pause(2)
%% Relative Yaw Angle vs Time

h3 = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
plot(e2_ampc.Values.Time,e2_ampc.Values.Data);
hold on;
grid on;
grid minor;
%title('\textbf{Relative Yaw Angle (e$_2$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Relative Yaw Angle [rad]','Interpreter','latex');
pos3 = get(h3,'Position');
set(h3,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos3(3)*0.8, pos3(4)-40])
print(h3,'figure\RelativeYawAngleVsTime','-dpdf','-r0')
hold off
pause(2)
%% Acceleration vs Time 

h4 = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
plot(accel_ampc.Values.time,accel_ampc.Values.Data);
hold on;
grid on;
grid minor;
%title('\textbf{Acceleration (u$_1$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Acceleration [m/s$^2$]','Interpreter','latex');
pos4 = get(h4,'Position');
set(h4,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos4(3)*0.8, pos4(4)-40])
print(h4,'figure\AccelerationVsTime','-dpdf','-r0')
hold off
pause(2)
%% Longitudinal Velocity

h5 = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
plot(vx_ampc.Values.Time,vx_ampc.Values.Data);
hold on;
grid on;
grid minor;
%title('\textbf{Velocity (V$_x$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Velocity [m/s]','Interpreter','latex');
pos5 = get(h5,'Position');
set(h5,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos5(3)*0.8, pos5(4)-40])
print(h5,'figure\LongitudinalVelocityVsTime','-dpdf','-r0')
hold off
pause(2)

%% Lateral Velocity

hpp = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
plot(vy_ampc.Values.Time,vy_ampc.Values.Data);
hold on;
grid on;
grid minor;
%title('\textbf{Velocity (V$_x$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Velocity [m/s]','Interpreter','latex');
pospp = get(hpp,'Position');
set(hpp,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pospp(3)*0.8, pospp(4)-40])
print(hpp,'figure\LateralVelocityVsTime','-dpdf','-r0')
hold off
pause(2)

%% Local function: Get data from simulation
function [e1,e2,delta,accel,vx,vy] = getData(logsout)
e1 = logsout.getElement('Lateral Deviation');    % lateral deviation
e2 = logsout.getElement('Relative Yaw Angle');   % relative yaw angle
delta = logsout.getElement('Steering');          % steering angle
accel = logsout.getElement('Acceleration');      % acceleration of ego car
vx = logsout.getElement('Longitudinal Velocity');% velocity of host car
vy = logsout.getElement('Lateral Velocity');