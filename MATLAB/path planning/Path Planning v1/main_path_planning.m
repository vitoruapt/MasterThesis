clear all
close all
clc

%% ATLASCAR2 - Moving Obstacle Avoidance based on MPC

car = struct;
road = struct;
obstacle = struct;

%% Vehicle model

% Parameters definition
car.length = 5;                 % Length of the ATLASCAR2
car.width = 2;                  % Witdh of the ATLASCAR2
car.V = 20;                     % Constant speed of the ATLASCAR2
car.x0 = [0; 0; pi/33; car.V];      % Initial conditions
car.u0 = [0; 0];
Ts = 0.02;                      % Sampling time

% Obtain a linear plant model at the nominal operating point and convert it
% into a discrete-time model to be used by the model predictive controller.
[Ad,Bd,Cd,Dd,U,Y,X,DX] = ATLASCAR2ModelDT(Ts,car.x0,car.u0);
                       
dsys = ss(Ad,Bd,Cd,Dd,'Ts',Ts);          % State space
dsys.InputName = {'Throttle','Delta'};   % Input u
dsys.StateName = {'X','Y','Theta','V'};  % State x
dsys.OutputName = dsys.StateName;        % Output y   

%% Plot initial condition

% ATLASCAR2 - Green with black boundary
% Horizontal lanes - Dashed black lines
% Obstacle/Car - Red with with black boundary
% Safe zone - Dashed red boundary.

f = obstaclePlotInitialCondition(car, obstacle, road);

%% MPC design

% MPC object: sampling time Ts, Prediction Horizon 10, Control Horizon 2
status = mpcverbosity('off');
mpcobj = mpc(dsys);

% Set the values of the prediction horizon and the control horizon
mpcobj.PredictionHorizon = 25;   % number of steps = 0.5 seconds
mpcobj.ControlHorizon = 5;

%% Constraints and Weights
% To prevent the ATLASCAR2 from accelerating or decelerating too quickly, 
% add a hard constraint of 0.2 (m^2/sec) on the throttle rate of change.
% mpcobj.ManipulatedVariables(1).RateMin = -0.2; 
% mpcobj.ManipulatedVariables(1).RateMax = 0.2;

% add a hard constraint of 6 degrees per sec on the steering angle rate of change.
mpcobj.ManipulatedVariables(2).RateMin = -pi/50;
mpcobj.ManipulatedVariables(2).RateMax = pi/50;

mpcobj.ManipulatedVariables(2).Min = -pi/30;
mpcobj.ManipulatedVariables(2).Max = pi/30;

% Scale the throttle and steering angle by their respective operating ranges.
%mpcobj.ManipulatedVariables(1).ScaleFactor = 2;
mpcobj.ManipulatedVariables(2).ScaleFactor = 0.2;

% Choose the Y position and velocity by setting the weights of the other 
% two outputs (X and theta) to zero
mpcobj.Weights.OV = [20 20 10 5];
mpcobj.Weight.MV = [0.2 10];

% Update the controller with the nominal operating condition
mpcobj.Model.Nominal = struct('U',U,'Y',Y,'X',X,'DX',DX);
%% Simulation
Duration = 70;
T = 0:Ts:Duration; 
x_ref = 150*sin(T/4);
y_ref = 300*sin(T/8);
x_dot = gradient(x_ref);
y_dot = gradient(y_ref);
theta_ref = atan2(x_dot,y_dot);
Vx=x_dot.*cos(theta_ref)-y_dot.*sin(theta_ref);
Vy=x_dot.*sin(theta_ref)+y_dot.*cos(theta_ref);
vel_ref=sqrt(Vx.^2+Vy.^2);
    
ref = [x_ref; y_ref; theta_ref; vel_ref]';               % Constant reference signal 

x = car.x0;                        % State initialization
u = car.u0;                        % Input initialization             
egoStates = mpcstate(mpcobj);      % Controller states inizialization

% Log simulation data for plotting
% saveSlope = zeros(length(T),1);         
% saveIntercept = zeros(length(T),1);
ympc = zeros(length(T),size(Cd,1));
umpc = zeros(length(T),size(Bd,2));

% Run the simulation
for k = 1:length(T)
    % Obtain new plant model and output measurements for interval |k|.
    [Ad,Bd,Cd,Dd,U,Y,X,DX] = ATLASCAR2ModelDT(Ts,x,u);
    measurements = Cd * x + Dd * u;
    ympc(k,:) = measurements';
    
    % Virtual safe zone around the obstacle
%    obstacle = obstacleGenerateObstacleGeometryInfo(obstacle);  
    
    % Determine whether the vehicle sees the obstacle, and update the mixed
    % I/O constraints when obstacle is detected.
%    detection = obstacleDetect(x,obstacle,road.laneWidth);
%    [E,F,G,saveSlope(k),saveIntercept(k)] = ...
%         obstacleComputeCustomConstraint(x,detection,obstacle,road.laneWidth,road.lanes); 
   
    % Prepare new plant model and nominal conditions for adaptive MPC.
    newPlant = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
    newNominal = struct('U',U,'Y',Y,'X',X,'DX',DX);
    
    % Prepare new mixed I/O constraints.
    options = mpcmoveopt;
   % options.CustomConstraint = struct('E',E,'F',F,'G',G);
    
    % Compute optimal moves using the updated plant, nominal conditions,
    % and constraints.
%     if (k<length(T)-25)
%         ref(k,:)=ref(k+25,:);
%     else
%         ref(k,:)=ref(end,:);
%     end
    [u,Info] = mpcmoveAdaptive(mpcobj,egoStates,newPlant,newNominal,...
        measurements,ref(k,:),[],options);
    umpc(k,:) = u';
    
    % Update the plant state for the next iteration |k+1|.
    x = Ad * x + Bd * u;
end

mpcverbosity(status);

%% Results
% Plot the trajectory of the ATLASCAR2 (blue line) and the third mixed
% I/O constraints (dashed green lines) during the obstacle avoidance
% maneuver.
 figure(f)
 plot(ref(:,1),ref(:,2))
 plot(ympc(:,1),ympc(:,2),'-k');
%     X = [0;50;100];
%     Y = saveSlope(k)*X + saveIntercept(k);
%     line(X,Y,'LineStyle','--','Color','g' )
% end    
% plot(ympc(:,1),ympc(:,2),'-b');
% axis([0 ympc(end,1) -road.laneWidth*road.lanes/2 road.laneWidth*road.lanes/2]) % reset axis

%% Animation
% writerObj = VideoWriter('animation.avi');
% writerObj.FrameRate = 1/Ts; 
% open(writerObj);
% 
% for k = 1:length(T) 
%     hold on 
%     grid on
%     grid minor
%     
%     % ATLASCAR2 green rectangle
%     p = patch([ympc(k,1)-car.length/2 ympc(k,1)-car.length/2 ympc(k,1)+car.length/2 ympc(k,1)+car.length/2], [ympc(k,2)-car.width/2, ympc(k,2)+car.width/2, ympc(k,2)+car.width/2, ympc(k,2)-car.width/2], [0 1 0]);
%     %rotate(p, [0 0 1], rad2deg(asin(ympc(k,3))), [ympc(k,1)-car.length/2 ympc(k,2)-car.width/2 0]);
% 
% %     frame = getframe(gcf); 
% %     writeVideo(writerObj, frame);
%     
%     pause(80/car.V/length(T))
%     delete(p)
% end
% % hold off
% % close(writerObj);
% %close(f);

