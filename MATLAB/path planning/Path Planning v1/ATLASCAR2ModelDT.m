function [Ad,Bd,Cd,Dd,U,Y,X,DX] = ATLASCAR2ModelDT(Ts,x,u)

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
Ac = [ 0, 0, -V*sin(theta), cos(theta);
       0, 0,  V*cos(theta), sin(theta);
       0, 0, 0,             tan(delta)/carLength;
       0, 0, 0,             0];
Bc = [0  , 0;
      0  , 0;
      0  , (V*(tan(delta)^2 + 1))/carLength;
      0.5, 0];
Cc = eye(4);
Dc = zeros(4,2);

% Generate discrete-time model (Ad, Bd, Cd, Dd)
% Use Simpson's rule to compute integral(0,Ts){expm(a*s)*ds*b}
n = 4;          % Number of points for Simpson's Rule, an even integer >= 2.
Ad = expm(Ac*Ts);
h = Ts/n;
Ai = eye(4) + Ad;
Coef = 2;
for i = 1:n-1
    if Coef == 2
        Coef = 4;
    else
        Coef = 2;
    end
    Ai = Ai + Coef*expm(Ac*i*h);
end
Bd = (h/3)*Ai*Bc;
Cd = Cc;
Dd = Dc;

% Nominal conditions for discrete-time plant
X = x;
U = u;
Y = x;
DX = Ad*x+Bd*u-x;