clear all
close all
clc

%% Lane Following Using Adaptive Model Predictive Control - Constant Longitudinal Velocity
mdl = 'lane_following_AMPC_constant_velocity';
open_system(mdl)

%%
% Define the sample time, |Ts|, and simulation duration, |T|, in seconds.
Ts = 0.02;
Duration = 15;

%%
% To describe the lateral vehicle dynamics, this example uses a _bicycle
% model_ with the following parameters:
%
% * |m| is the total vehicle mass (kg)
% * |Iz| is the yaw moment of inertia of the vehicle (mNs^2).
% * |lf| is the longitudinal distance from the center of gravity to the
% front tires (m).
% * |lr| is the longitudinal distance from center of gravity to the rear
% tires (m).
% * |Cf| is the cornering stiffness of the front tires (N/rad).
% * |Cr| is the cornering stiffness of the rear tires (N/rad).
%
m = 1575;
Iz = 2875;
lf = 1.2;
lr = 1.6;
Cf = 19000;
Cr = 33000;

%%
% You can represent the lateral vehicle dynamics using a linear
% time-invariant (LTI) system with the following state, input, and output
% variables. The initial conditions for the state variables are assumed to
% be zero.
%
% * State variables: Lateral velocity $V_y$ and yaw angle rate $r$
% * Input variable: Front steering angle $\delta$
% * Output variables: Same as state variables

%%
% In this example, the longitudinal vehicle dynamics are separated from the
% lateral vehicle dynamics. Therefore, the longitudinal velocity is assumed
% to be constant. In practice, the longitudinal velocity can vary and the
% Lane Keeping Assist System Block uses adaptive MPC to adjust the model of
% the lateral dynamics accordingly.

% Specify the longitudinal velocity in m/s.
Vx = 20;

%%
% Specify a state-space model, |G(s)|, of the lateral vehicle dynamics.
A = [-(2*Cf+2*Cr)/m/Vx, -Vx-(2*Cf*lf-2*Cr*lr)/m/Vx;...
     -(2*Cf*lf-2*Cr*lr)/Iz/Vx, -(2*Cf*lf^2+2*Cr*lr^2)/Iz/Vx];
B = [2*Cf/m, 2*Cf*lf/Iz]';
C = eye(2);
G = ss(A,B,C,0);

%% Sensor Dynamics and Curvature Previewer
% In this example, the Sensor Dynamics block outputs the lateral deviation
% and relative yaw angle. The dynamics for relative yaw angle are
% $$\dot{e}_2 = r-V_x\rho$, where $\rho$ denotes the curvature. The
% dynamics for lateral deviation are $\dot{e}_1 = V_x e_2+V_y$.

%%
% The Curvature Previewer block outputs the previewed curvature with a
% look-ahead time one second. Therefore, given a sample time $Ts = 0.02$,
% the prediction horizon |25| steps. The curvature used in this example is
% calculated based on trajectories for a double lane change maneuver.

%%
% Specify the prediction horizon and obtain the previewed curvature.
PredictionHorizon = 25;

time = 0:Ts:Duration;
md = getCurvature(Vx,time);


%% Configuration of the Lane Keeping Assist System Block
% The LKA system is modeled in Simulink using the Lane Keeping Assist
% System block. The inputs to the LKA system block are:
%
% * Previewed curvature     (from lane detections)
% * Ego longitudinal velocity
% * Lateral deviation       (from lane detections)
% * Relative yaw angle      (from lane detections)
%
% The output of the LKA system is the front steering angle of the ego car.
% Considering the physical limitations of the ego car, the steering angle
% is constrained to the range [-0.5,0.5] rad/s.

u_min = -0.5;
u_max = 0.5;

%%
% For this example, the default parameters of the Lane Keeping Assist
% System block match the simulation parameters. If your simulation
% parameters differ from the default values, then update the block
% parameters accordingly.

%% Simulation Analysis
% Run the model.
sim(mdl)

%%
% Plot the simulation results.
lane_following_constant_velocity_Results(logsout)

