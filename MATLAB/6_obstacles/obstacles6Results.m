%% Plot results for adaptive MPC

% Velocity vs Time
h1 = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
plot(T,ympc(:,4));
hold on;
grid on;
grid minor;
%title('\textbf{Velocity ($v$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Velocity [m/s]','Interpreter','latex');
pos1 = get(h1,'Position');
set(h1,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos1(3)*0.8, pos1(4)-40])
print(h1,'figure\VelocityVsTime','-dpdf','-r0')
hold off
pause(2)

% LateralPosition vs Time
h2 = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
plot(T,ympc(:,2));
hold on;
grid on;
grid minor;
%title('\textbf{Lateral Position ($y$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Lateral Position [m]','Interpreter','latex');
pos2 = get(h2,'Position');
set(h2,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos2(3)*0.8, pos2(4)-40])
print(h2,'figure\LateralPositionVsTime','-dpdf','-r0')
hold off
pause(2)

% Theta vs Time
h3 = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
plot(T,ympc(:,3));
hold on;
grid on;
grid minor;
%title('\textbf{Heading Angle($\theta$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Heading Angle [rad]','Interpreter','latex');
pos3 = get(h3,'Position');
set(h3,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos3(3)*0.8, pos3(4)-40])
print(h3,'figure\HeadingAngleVsTime','-dpdf','-r0')
hold off
pause(2)

% Throttle vs Time
h5 = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
plot(T,umpc(:,1));
hold on;
grid on;
grid minor;
%title('\textbf{Throttle($T$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Throttle [m/s$^2$]','Interpreter','latex');
pos5 = get(h5,'Position');
set(h5,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos5(3)*0.8, pos5(4)-40])
print(h5,'figure\ThrottleVsTime','-dpdf','-r0')
hold off
pause(2)

% Steering Angle vs Time
h6 = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
plot(T,umpc(:,2));
hold on;
grid on;
grid minor;
%title('\textbf{Steering Angle($\delta$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Steering Angle[rad]','Interpreter','latex');
pos6 = get(h6,'Position');
set(h6,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos6(3)*0.8, pos6(4)-40])
print(h6,'figure\SteeringAngleVsTime','-dpdf','-r0')
hold off
pause(2)

% LateralPosition vs Time
h7 = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
plot(T,ympc(:,1));
hold on;
grid on;
grid minor;
%title('\textbf{Lateral Position ($y$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Longitudinal Position [m]','Interpreter','latex');
pos7 = get(h7,'Position');
set(h7,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos7(3)*0.8, pos7(4)-40])
print(h7,'figure\LongitudinalPositionVsTime','-dpdf','-r0')
hold off
pause(2)