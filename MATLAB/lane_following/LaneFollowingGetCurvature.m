function rho = LaneFollowingGetCurvature(Vx,time)
% Get previewed curvature from desired X and Y positions for LKA
%
% Inputs:
%   Vx: longitudinal velocity
%   time: time vector
%
% Outputs:
%   rho: previewed curvature

% Desired X position
Xref = Vx*time;

% Desired Y position
z1 = (2.4/50)*(Xref-27.19)-1.2;
z2 = (2.4/43.9)*(Xref-56.46)-1.2;
Yref = 5*sin(Xref/(20)) ;%(2.4/50)*(Xref+3).^2 - (2.4/43.9)*(Xref-12).^2; %8.1/2*(1+tanh(z1)) - 11.4/2*(1+tanh(z2));  %z1.^2 - z2.^2;  

h = figure ('Position',[100, 100, 340, 230], 'PaperPositionMode','auto');
plot(Xref,Yref)
hold on
grid on
grid minor
axis equal
title('\textbf{Desired Path}','Interpreter','latex');
xlabel('X-coordinate [m]','Interpreter','latex');
ylabel('Y-coordinate [m]','Interpreter','latex');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos(3)*0.8, pos(4)-40])
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
set(hcurv,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos(3)*0.8, pos(4)-40])
print(hcurv,'figure\Curvature','-dpdf','-r0')
hold off
hold off