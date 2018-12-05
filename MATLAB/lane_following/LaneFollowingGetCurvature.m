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
Yref = 5*sin(Xref/(20));%(2.4/50)*(Xref+3).^2 - (2.4/43.9)*(Xref-12).^2; %8.1/2*(1+tanh(z1)) - 11.4/2*(1+tanh(z2));  %z1.^2 - z2.^2;  

figure()
plot(Xref,Yref)
hold on
grid on
grid minor
axis equal
title('\textbf{Desired Path}','Interpreter','latex');
xlabel('X-coordinate [m]','Interpreter','latex');
ylabel('Y-coordinate [m]','Interpreter','latex');


% Desired curvature
DX = gradient(Xref,0.1);
DY = gradient(Yref,0.1);
D2Y = gradient(DY,0.1);
curvature = DX.*D2Y./(DX.^2+DY.^2).^(3/2);

% Stored curvature (as input for LKA)
rho.time = time;
rho.signals.values = curvature';