
% This is the script to design an adaptive MPCcontroller for lane following. 
% MPC controller has the following structure:
% 
%   MV: acceleration and steering
%   DV: curvature (with look-ahead)
%   OV: longitudinal velocity, lateral deviation and relative yaw angle
status = mpcverbosity('off');

%% Longitudinal dynamics
sys_longitudinal = ss(tf(1,[tau,1,0]));

%% Lateral dynamics
[Ag,Bg,Cg] = getVehModelFromParam(m,Iz,lf,lr,Cf,Cr,v0);
% Constants
n = size(Bg,1); % number of states for vehicle model                                              
C = [zeros(2,n),eye(2)];
D = zeros(2);
% Augement the vehicle model into the MPC internal model
[A,B] = getFullModel(Ag,Bg,Cg,v0);
sys_lateral = ss(A,B,C,D);

%% MPC internal model
sys = [sys_longitudinal,zeros(1,2);zeros(2,1),sys_lateral];
sys = minreal(sys);

%% MPC design 
sys = setmpcsignals(sys,'MV',[1,2],'MD',3); % acceleration, steering and desired yaw rate
sysd = c2d(sys,Ts);
mpc1 = mpc(sysd,Ts);

% Specify the prediction horizon.
mpc1.PredictionHorizon = 25;
% Specify MV limits.
mpc1.MV(1).Min = -3;
mpc1.MV(1).Max = 3;
mpc1.MV(2).Min = -1.13;
mpc1.MV(2).Max = 1.13;
% Specify nominal values.
mpc1.Model.Nominal.Y(1) = v0; 
mpc1.OV(1).ScaleFactor = 15;   % Typical value of longitudinal velocity
mpc1.OV(2).ScaleFactor = 0.5;  % Range for lateral deviation
mpc1.OV(3).ScaleFactor = 0.5;  % Range for relative yaw angle
mpc1.MV(1).ScaleFactor = 6; % Range of steering angle
mpc1.MV(2).ScaleFactor = 2.26;    % Range of acceleration
mpc1.DV.ScaleFactor = 0.2;     % Range of Curvature
% Specify weights.
alpha = 1; % non-zero
mpc1.Weights.MVRate = [0.3 0.1]*alpha;
mpc1.Weights.OV = [1 1 0]/alpha; 

%% Get the model for adaptive MPC
[A,B,C,D] = LaneFollowingGetModelForAMPC(Ts,v0,Ag,Bg,Cg);

%% Local function: Get continuous vehicle lateral model from parameters
function [Ag,Bg,Cg] = getVehModelFromParam(m,Iz,lf,lr,Cf,Cr,Vx)
% Get continuous vehicle model from vehicle parameters with steering angle
% as input. The first output is the lateral velocity and the second
% output is the yaw angle rate.

% Specify vehicle state-space model with state varaibles [vy,phidot]. 
%   vy:       lateral velocity
%   phidot:   yaw angle rate
%   dynamics: dx = [a1,a2;a3,a4]x + [b1;b2]u.
a1 = -(2*Cf+2*Cr)/m/Vx;
a2 = -(2*Cf*lf-2*Cr*lr)/m/Vx - Vx;
a3 = -(2*Cf*lf-2*Cr*lr)/Iz/Vx;
a4 = -(2*Cf*lf^2+2*Cr*lr^2)/Iz/Vx;
b1 = 2*Cf/m;
b2 = 2*Cf*lf/Iz;
% Specify the vehicle model matrices with states [vy,phidot].
Ag = [a1,a2;a3,a4];
Bg = [b1;b2];
Cg = eye(2);
end

%% Local functions: Augment the vehicle model into the MPC internal model
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
end