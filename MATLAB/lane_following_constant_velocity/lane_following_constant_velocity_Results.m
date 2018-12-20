function lane_following_constant_velocity_Results(logsout)
%% Get the data from simulation
steering_angle = logsout.getElement('steering_angle');
lateral_deviation = logsout.getElement('lateral_deviation');
relative_yaw_angle = logsout.getElement('relative_yaw_angle');

%% Lateral deviation vs Time

h1 = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
hold on;
grid on;
grid minor;
plot(lateral_deviation.Values.time,lateral_deviation.Values.Data,'b')
title('\textbf{Lateral Deviation (e$_1$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Lateral Deviation [m]','Interpreter','latex');
pos = get(h1,'Position');
set(h1,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos(3), pos(4)-41])
print(h1,'figure\LateralDeviationVsTime','-dpdf','-r0')
hold off
pause(2)

%% Relative Yaw Angle vs Time

h2 = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
hold on;
grid on;
grid minor;
plot(relative_yaw_angle.Values.time,relative_yaw_angle.Values.Data,'b')
title('\textbf{Relative Yaw Angle (e$_2$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Relative Yaw Angle [rad]','Interpreter','latex');
pos = get(h2,'Position');
set(h2,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos(3), pos(4)-40])
print(h2,'figure\RelativeYawAngleVsTime','-dpdf','-r0')
hold off
pause(2)

%% Steering angle vs Time
h3 = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
hold on;
grid on;
grid minor;
plot(steering_angle.Values.time,steering_angle.Values.Data,'b')
title('\textbf{Steering Angle (u$_2$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Steering angle [rad]','Interpreter','latex');
pos = get(h3,'Position');
set(h3,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos(3), pos(4)-40])
print(h3,'figure\SteeringAngleVsTime','-dpdf','-r0')
hold off
