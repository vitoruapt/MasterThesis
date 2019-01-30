function rho = LaneFollowingGetCurvature(Vx,time)
% Get previewed curvature from desired X and Y positions for LKA
%
% Inputs:
%   Vx: longitudinal velocity
%   time: time vector
%
% Outputs:
%   rho: previewed curvature

%Xref=22*cos(Vx*time/60);
%Yref=20*sin(Vx*time/60);

% Desired X position

Xref = Vx*time;
%Xref=200*cos(Vx*time/60);
%Yref=100*sin(Vx*time/60);
% Desired Y position
z1 = (2.4/50)*(Xref-27.19)-1.2;
z2 = (2.4/43.9)*(Xref-56.46)-1.2;
Yref = 5*sin(Xref/(20)); % Correct reference sinusoidal
%Yref = (2.4/50)*(Xref+3).^2 - (2.4/43.9)*(Xref-12).^2; % Path Parabola
%Yref = 8.1/2*(1+tanh(z1)) - 11.4/2*(1+tanh(z2)); % Avoid One obstacle

h = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
plot(Xref,Yref)
hold on
grid on
grid minor
title('\textbf{Desired Path}','Interpreter','latex');
xlabel('X-coordinate [m]','Interpreter','latex');
ylabel('Y-coordinate [m]','Interpreter','latex');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos(3), pos(4)-40])
print(h,'figure\Reference','-dpdf','-r0')
hold off

% Desired curvature
DX = gradient(Xref,0.1);
DY = gradient(Yref,0.1);
D2Y = gradient(DY,0.1);
curvature = DX.*D2Y./(DX.^2+DY.^2).^(3/2);

% Stored curvature (as input for LKA)
rho.time = time;
rho.signals.values = curvature';

hcurv = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
plot(rho.time,rho.signals.values)
hold on
grid on
grid minor
title('\textbf{Curvature}','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Curvature [m]','Interpreter','latex');
pos = get(hcurv,'Position');
set(hcurv,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos(3), pos(4)-40])
print(hcurv,'figure\Curvature','-dpdf','-r0')
hold off