function [A,B,C,D] = LaneFollowingGetModelForAMPC(Ts,Vx,Ag,Bg,Cg)

% Get discrete state-space model for Adaptive MPC. 

% Inputs:
%   Ts:                mask variable for sample time
%   Vx:                mask inport for longitudinal velocity
%   Ag,Bg,Cg:          continuous or discrete vehicle model with sample time Ts        
% Outputs:
%   A,B,C,D:           discrete time model for MPC

% Get full model for MPC
%   Vehicle model (continuous or discrete) with input delta (steering
%   angle) and outputs [vy,phidot] (lateral velocity and yaw angle rate).
%
%   The first step is to get model Gp from input [delta,phidotdes] to
%   [vy,phidot,phidotdes] with the desired yaw angle rate phidotdes.
%
%   The second step is to get continuous model Ge from
%   [vy,phidot,phidotdes] to [e1,e2] (lateral error and yaw angle error).
%   If a discrete-time vehicle model is specified, then Ge must be
%   discretized as well.
%
%   The overall model Gf is the series connection of Gp and Ge, from input
%   [delta,phidotdes] to output [e1,e2]. If continuous-time vehicle is
%   specified, then the model Gf need to be discretized as well. 


% Lateral model.
[A2,B2] = getFullModel(Ag,Bg,Cg,Vx);
n = size(Bg,1); % number of states for vehicle model                                              
C2 = [zeros(2,n),eye(2)];

% Longitudinal model.
A1 = [-2,0;1,0]; 
B1 = [2;0]; 
C1 = [0,1];

% Combined model.
Af = [A1,zeros(2,n+2);zeros(n+2,2),A2];
Bf = [B1,zeros(2);zeros(4,1),B2];
C = [C1,zeros(1,4);zeros(2),C2];
D = zeros(3);

% Convert continuous to discrete time
[A, B] = getDiscrete(Af, Bf, Ts); 


%% Local function: Augement the vehicle model into the MPC internal model
function [Af,Bf] = getFullModel(Ag,Bg,Cg,Vx)
% dynamics for Gp from [delta,phidotdes] to [vy,phidot,phidotdes]
Ap = Ag;
n = size(Bg,1);                                                
Bp = [Bg,zeros(n,1)];
Cp = [Cg;zeros(1,n)];
Dp = [zeros(2);0,1];

% continuous dynamics for Ge from [vy,phidot,phidotdes] to [e1,e2]
Ae = [0,Vx;0,0];
Be = [1,0,0;0,1,-1];

% series connection of Gp and Ge
Af = [Ap,zeros(n,2);Be*Cp,Ae];
Bf = [Bp;Be*Dp];

%% Local function: Discretize model with sample time Ts
function [A, B] = getDiscrete(a, b, Ts)
% Convert to discrete time
A = expm(a*Ts);
nx = size(b,1);
n = 4;  % Number of points for Simpson's Rule, an even integer >= 2.
% Use Simpson's rule to compute integral(0,Ts){expm(a*s)*ds*b}
h = Ts/n;
Ai = eye(nx) + A;        % First and last terms
Coef = 2;
for i = 1:n-1
    if Coef == 2
        Coef = 4;
    else
        Coef = 2;
    end
    Ai = Ai + Coef*expm(a*i*h);     % Intermediate terms
end
B = (h/3)*Ai*b;
