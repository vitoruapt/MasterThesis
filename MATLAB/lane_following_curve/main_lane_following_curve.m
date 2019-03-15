clear all
close all
clc

%% Lane Following Using Adaptive Model Predictive Control
mdl = 'lane_following_AMPC';
open_system(mdl)

% Parameters of Vehicle Dynamics and Road Curvature. 
% Specify the vehicle dynamics parameters
m = 1575;   % Mass of ATLASCAR2
Iz = 2875;  % Moment of inertia about Z axis
lf = 1.2;   % Distance between Center of Gravity and Front axle 
lr = 1.6;   % Distance between Center of Gravity and Rear axle
Cf = 19000; % Cornering stiffness of the front tires (N/rad)
Cr = 33000; % Cornering stiffness of the rear tires (N/rad).
tau = 0.2;  % Time constant

% Set the initial and driver-set velocities.
v0 = 15;    % Initial velocity
v_set = 20; % Driver set velocity

% Set the controller sample time.
Ts = 0.02;

% Obtain the lane curvature information seconds.
Duration = 10;                              % Simulation duration
t = 0:Ts:Duration;                          % Time vector
[rho,Yref] = LaneFollowingGetCurvature(v_set,t);   % Signal containing curvature information

%% Design Adaptive Model Predictive Controller 
% This controller uses a linear model for the vehicle dynamics and updates
% the model online as the longitudinal velocity varies. The adaptive MPC
% controller is designed in |LaneFollowingGenerateAMPC| and the model is
% updated using |LaneFollowingGetModelForAMPC|.
LaneFollowingGenerateAMPC;

%% Simulation
% Simulate the model using adaptive MPC and plot the results using the 
% function |LaneFollowingResults|
sim(mdl)
logsout1 = logsout;
LaneFollowingResults(logsout1,Yref)
print -slane_following_AMPC -dpdf lane_following_AMPC.pdf
movefile('lane_following_AMPC.pdf','figure')

print -slane_following_AMPC/Sensor_Dynamics -dpdf lane_following_AMPC_sensor_dynamics.pdf
movefile('lane_following_AMPC_sensor_dynamics.pdf','figure')

print -slane_following_AMPC/Vehicle_Dynamics -dpdf lane_following_AMPC_vehicle_dynamics.pdf
movefile('lane_following_AMPC_vehicle_dynamics.pdf','figure')