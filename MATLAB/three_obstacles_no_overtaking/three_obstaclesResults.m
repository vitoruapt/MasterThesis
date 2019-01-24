%% Plot results for adaptive MPC

% Velocity vs Time
h1 = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
hold on;
grid on;
grid minor;
plot(T,ympc(:,4));
title('\textbf{Velocity ($v$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Velocity [m/s]','Interpreter','latex');
pos = get(h1,'Position');
set(h1,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos(3), pos(4)-40])
print(h1,'figure\VelocityVsTime','-dpdf','-r0')
hold off

% Y_mpc vs Time
h2 = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
hold on;
grid on;
grid minor;
plot(T,ympc(:,2));
title('\textbf{Lateral Position ($y$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Lateral Position [m]','Interpreter','latex');
pos = get(h1,'Position');
set(h2,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos(3), pos(4)-40])
print(h2,'figure\LateralPositionVsTime','-dpdf','-r0')
hold off

% Theta vs Time
h3 = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
hold on;
grid on;
grid minor;
plot(T,ympc(:,3));
title('\textbf{Heading Angle ($\theta$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Heading Angle [rad]','Interpreter','latex');
pos = get(h1,'Position');
set(h3,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos(3), pos(4)-40])
print(h3,'figure\HeadingAngleVsTime','-dpdf','-r0')
hold off

% Throttle vs Time
h4 = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
hold on;
grid on;
grid minor;
plot(T,umpc(:,1));
title('\textbf{Throttle ($T$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Throttle [$m^2/s$]','Interpreter','latex');
pos = get(h1,'Position');
set(h4,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos(3), pos(4)-40])
print(h4,'figure\ThrottleVsTime','-dpdf','-r0')
hold off

% Steering Angle vs Time
h5 = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
hold on;
grid on;
grid minor;
plot(T,umpc(:,2));
title('\textbf{Steering Angle ($\delta$) vs Time}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Steering Angle [rad]','Interpreter','latex');
pos = get(h1,'Position');
set(h5,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos(3), pos(4)-40])
print(h5,'figure\SteeringAngleVsTime','-dpdf','-r0')
hold off