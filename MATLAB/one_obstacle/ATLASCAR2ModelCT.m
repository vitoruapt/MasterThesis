function [xdot,y] = ATLASCAR2ModelCT(x,u)

% The ATLASCAR2 has rectangular shaper with a length of 5 meters and width of
% 2 meters. The model has four states:
%
% * |xPos| - Global horizontal position of the car center
% * |yPos| - Global vertical position of the car center
% * |theta| - Heading angle of the car (0 when facing east, counterclockwise positive)
% * |V| - Speed of the car (positve)
%
% There are two manipulated variables:
%
% * |throttle| - Throttle (positive when accelerating, negative when braking)
% * |delta| - Steering angle change (counterclockwise positive)

% Inizialization
carLength = 5;  % Length of the ATLASCAR2
theta = x(3);   % Heading angle of the ATLASCAR2 (0 when facing east, counterclockwise positive) 
V = x(4);       % Speed of the ATLASCAR2 (positve)
delta = u(2);   % Steering angle change (counterclockwise positive)

% Generate continuous-time model (Ac,Bc,Cc,Dc)
A = [ 0, 0, -V*sin(theta), cos(theta);
      0, 0,  V*cos(theta), sin(theta);
      0, 0, 0,             tan(delta)/carLength;
      0, 0, 0,             0];
B = [0  , 0;
     0  , 0;
     0  , (V*(tan(delta)^2 + 1))/carLength;
     0.5, 0];
C = eye(4);
D = zeros(4,2);
xdot = A*x + B*u;
y = C*x + D*u;