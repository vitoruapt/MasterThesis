clear all
close all
clc

%% ATLASCAR2 - Moving Obstacle Avoidance based on MPC

car = struct;
road = struct;
obstacle = struct;

%% Vehicle model

% Parameters definition

road.lanes = 3;                         % The road is straight and has 3 lanes.
road.laneWidth = 4;                     % Each lane is |4| meters wide.
road.width = road.laneWidth*road.lanes; % Width of the street 12 meters
road.centre_x0 = 0;                   
road.centre_y0 = 0;
road.external_rx = 300;    
road.external_ry = 110;
road.internal_rx = road.external_rx - road.width;
road.internal_ry = road.external_ry - road.width;
road.centre_rx=road.external_rx-(road.width)/2;
road.centre_ry=road.external_ry-(road.width)/2;

Ts = 0.02; 
car.length = 5;                 % Length of the ATLASCAR2
car.width = 2;                  % Witdh of the ATLASCAR2
car.V = 20;                     % Constant speed of the ATLASCAR2
car.angle_init = pi/5;                                
car.x_init = road.centre_x0+road.centre_rx*cos(car.angle_init*Ts/35);
car.y_init = road.centre_y0+road.centre_ry*sin(car.angle_init*Ts/35);
car.x0 = [car.x_init; car.y_init; 0; car.V];            % Initial conditions
init_slope = -(road.centre_rx*cos(car.angle_init)*(road.centre_ry^2))/(road.centre_ry*sin(car.angle_init)*(road.centre_rx^2));
init_delta =  atan2(init_slope,1);
car.u0 = [0; init_delta];
                     

% Obtain a linear plant model at the nominal operating point and convert it
% into a discrete-time model to be used by the model predictive controller.
[Ad,Bd,Cd,Dd,U,Y,X,DX] = ATLASCAR2ModelDT(Ts,car.x0,car.u0);

dsys = ss(Ad,Bd,Cd,Dd,'Ts',Ts);          % State space
dsys.InputName = {'Throttle','Delta'};   % Input u
dsys.StateName = {'X','Y','Theta','V'};  % State x
dsys.OutputName = dsys.StateName;        % Output y   

%% Road and Obstacles Information

N = 1; %randi(6,1,1);          % Random number of obstacles (1,2,3,4,5,6)

for i=1:N
    obstacle(i).length = 5;     % The obstacles in this example are moving cars 
    obstacle(i).width = 2;      % with the same size and shape of the ATLASCAR2
    
    obstacle(i).th = pi/2+i*pi/7;
    
    obstacle(i).X = road.centre_x0+road.centre_rx*cos(obstacle(i).th);         % Initial positions of the N obstacles   
    obstacle(i).Y = road.centre_y0+road.centre_ry*sin(obstacle(i).th);
   
    obstacle(i).safeDistanceX = obstacle(i).length;    % Length equal to two car lengths
    obstacle(i).safeDistanceY = road.laneWidth;        % Width equal to two lane widths

    % Assume that the LIDAR sensor can detect an obstacle 30
    % meters in front of the ATLASCAR2.
    obstacle(i).DetectionDistance = 30;
end

% Virtual safe zone around the obstacle

obstacle = obstacleGenerateObstacleGeometryInfo(obstacle, N);

%% Plot initial condition

% ATLASCAR2 - Green with black boundary
% Horizontal lanes - Dashed black lines
% Obstacle/Car - Red with with black boundary
% Safe zone - Dashed red boundary.

f = obstaclePlotInitialCondition(car, obstacle, road, N);

%% MPC design
 
% MPC object: sampling time Ts, Prediction Horizon 10, Control Horizon 2 (Default)
status = mpcverbosity('off');
mpcobj = mpc(dsys);

% Set the values of the prediction horizon and the control horizon
mpcobj.PredictionHorizon = 25;    % number of steps = 0.5 seconds
mpcobj.ControlHorizon = 5;

%% Constraints and Weights

% To prevent the ATLASCAR2 from accelerating or decelerating too quickly, 
% add a hard constraint of 0.2 (m^2/sec) on the throttle rate of change.
mpcobj.ManipulatedVariables(1).RateMin = -0.2; 
mpcobj.ManipulatedVariables(1).RateMax = 0.2;

% add a hard constraint of 6 degrees per sec on the steering angle rate of change.
mpcobj.ManipulatedVariables(2).RateMin = -pi/30;
mpcobj.ManipulatedVariables(2).RateMax = pi/30;

% Scale the throttle and steering angle by their respective operating ranges.
mpcobj.ManipulatedVariables(1).ScaleFactor = 2;
mpcobj.ManipulatedVariables(2).ScaleFactor = 0.2;
 
% Choose the Y position and velocity by setting the weights of the other 
% two outputs (X and theta) to zero
mpcobj.Weights.OutputVariables = [1 1 0 1];

% Update the controller with the nominal operating condition
mpcobj.Model.Nominal = struct('U',U,'Y',Y,'X',X,'DX',DX);
% 
%% Specify Mixed I/O Constraints for Obstacle Avoidance Maneuver

% E*u + F*y <= G where u is the manipulated variable vector and y is the output
% variable vector. You can update the constraint matrices E, F, and G when 
% the controller is running.
% The first constraint is an upper bound on y

t = linspace(1,360,361) ;
constraint_x_iniz_int = -(road.centre_x0 + road.internal_rx*cos(t));
constraint_y_iniz_int = -(road.centre_y0 + road.internal_ry*sin(t));

constraint_x_iniz_ext = road.centre_x0 + road.external_rx*cos(t);
constraint_y_iniz_ext = road.centre_y0 + road.external_ry*sin(t);

constraint_x_iniz = [constraint_x_iniz_int, constraint_x_iniz_ext];
constraint_y_iniz = [constraint_y_iniz_int, constraint_y_iniz_ext];

E1 = [0 0];
F1 = [0 1 0 0];
G1 = [constraint_y_iniz(2)];
E4 = [0 0];
F4 = [1 0 0 0];
G4 = [constraint_x_iniz(2)];
% The second constraint is a lower bound on y
E2 = [0 0];
F2 = [0 -1 0 0]; 
G2 = [-constraint_y_iniz(1)];
E5 = [0 0];
F5 = [-1 0 0 0];
G5 = [-constraint_x_iniz(1)];
% The third constraint is for obstacle avoidance. Even though no obstacle
% is detected at the nominal operating condition, you must add a "fake"
% constraint here because you cannot change the dimensions of the
% constraint matrices at run time. For the fake constraint, use a
% % constraint with the same form as the second constraint.
 E3 = [0 0];
 F3 = [0 -1 0 0];
 G3 = [-constraint_y_iniz(1)];
 E6 = [0 0];
 F6 = [-1 0 0 0];
 G6 = [-constraint_x_iniz(1)];
 
 % Specify the mixed input/output constraints in the controller using the
% |setconstraint| function.
    setconstraint(mpcobj,[E1;E2;E3;E4;E5;E6],[F1;F2;F3;F4;F5;F6],[G1;G2;G3;G4;G5;G6]);
    
%% Simulation

x = car.x0;                        % State initialization
u = car.u0;                        % Input initialization             
egoStates = mpcstate(mpcobj);      % Controller states inizialization
 
T = 0:Ts:10;                        % Simulation time
 
% Log simulation data for plotting
saveSlope = zeros(length(T),N);         
saveIntercept = zeros(length(T),N);
ympc = zeros(length(T),size(Cd,1));
umpc = zeros(length(T),size(Bd,2));  

car.velocity_angle(1)=pi/6;
% Run the simulation
for k = 1:length(T) 
    % Obtain new plant model and output measurements for interval |k|.
    ref(k,:) = [road.centre_x0+road.centre_rx*cos(car.velocity_angle(k)*Ts/35) road.centre_y0+road.centre_ry*sin(car.velocity_angle(k)*Ts/35) 0 car.V];
    h=(road.centre_rx-road.centre_ry)/(road.centre_rx+road.centre_ry); 
    h=3.0*h; 
    l=pi*(road.centre_rx+road.centre_ry)*(1.0+(h/(10.0+sqrt(4.0-h))));
    xx(k) = road.centre_x0+road.centre_rx*sin(car.velocity_angle(k));
    yy(k) = road.centre_y0+road.centre_ry*cos(car.velocity_angle(k));
    da(k) = l/sqrt((xx(k).*xx(k))+(yy(k).*yy(k)));
    car.velocity_angle(k+1)=car.velocity_angle(k)-da(k);
   
    [Ad,Bd,Cd,Dd,U,Y,X,DX] = ATLASCAR2ModelDT(Ts,x,u);
    measurements = Cd * x + Dd * u;
    ympc(k,:) = measurements';
 
    % Create Obstacle Dynamics
    for i=1:N  
        obstacle(i).velocity_angle(k)=k*pi/130;
        obstacle(i).X(k+1) = road.centre_x0+road.centre_rx*cos(obstacle(i).th+obstacle(i).velocity_angle(k));
        obstacle(i).Y(k+1) = road.centre_y0+road.centre_ry*sin(obstacle(i).th+obstacle(i).velocity_angle(k));
    % Safe zones for the plot
        flSafeX(k+1,i) = obstacle(i).X(k+1)+obstacle(i).safeDistanceX;
        frSafeX(k+1,i) = obstacle(i).X(k+1)+obstacle(i).safeDistanceX;
        rlSafeX(k+1,i) = obstacle(i).X(k+1)-obstacle(i).safeDistanceX; 
        rrSafeX(k+1,i) = obstacle(i).X(k+1)-obstacle(i).safeDistanceX;
        
        flSafeY(k+1,i) = obstacle(i).Y(k+1)-obstacle(i).safeDistanceY;
        frSafeY(k+1,i) = obstacle(i).Y(k+1)+obstacle(i).safeDistanceY;
        rlSafeY(k+1,i) = obstacle(i).Y(k+1)+obstacle(i).safeDistanceY; 
        rrSafeY(k+1,i) = obstacle(i).Y(k+1)-obstacle(i).safeDistanceY; 
    end

    % Virtual safe zone around the obstacle
    obstacle = obstacleGenerateObstacleGeometryInfo(obstacle,N);  
     
    % Determine whether the vehicle sees the obstacle, and update the mixed
    % I/O constraints when obstacle is detected.
    [detection(k,:) m]= obstacleDetect(x,obstacle,road,N,car);
    [E,F,G,saveSlope(k,:),constraint_x(k,:), constraint_y(k,:)] = ...
    obstacleComputeCustomConstraint(x,detection(k,:),obstacle,road.laneWidth,road.lanes,N,m,road,car); 
    
  % Prepare new plant model and nominal conditions for adaptive MPC.
    newPlant = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
    newNominal = struct('U',U,'Y',Y,'X',X,'DX',DX);
     
  % Prepare new mixed I/O constraints.
  options = mpcmoveopt;
  %options.CustomConstraint = struct('E',E,'F',F,'G',G);
    
  % Compute optimal moves using the updated plant, nominal conditions,
  % and constraints.
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
% maneuver (only for the first obstacle)
figure(f)
% for k = 1:length(saveSlope)
%     X = [0;50;100];
%     Y = saveSlope(k)*X + saveIntercept(k);
%     line(X,Y,'LineStyle','--','Color','g' )  
% end


%% Animation
% writerObj = VideoWriter('animation.avi'); % Name it.
% writerObj.FrameRate = 1/Ts; % How many frames per second.
% open(writerObj);
  
for k = 1:length(T) 
    hold on 
    grid on
    grid minor
    axis equal

   % ATLASCAR2 green rectangle
   refp = patch([ref(k,1)-car.length/2 ref(k,1)-car.length/2 ref(k,1)+car.length/2 ref(k,1)+car.length/2],...
        [ref(k,2)-car.width/2, ref(k,2)+car.width/2, ref(k,2)+car.width/2, ref(k,2)-car.width/2], [0 1 0]);
     slope(k)=-(road.centre_rx*cos(car.velocity_angle(k)*Ts/35)*(road.centre_ry^2))/(road.centre_ry*sin(car.velocity_angle(k)*Ts/35)*(road.centre_rx^2));
     angle_turn(k) = atan(slope(k));
     rotate(refp, [0 0 1], rad2deg(angle_turn(k)), [ref(k,1) ref(k,2) 0]);
   %line([constraint_x(k,1) constraint_x(k,2)],[constraint_y(k,1) constraint_y(k,2)]);
   
     p = patch([ympc(k,1)-car.length/2 ympc(k,1)-car.length/2 ympc(k,1)+car.length/2 ympc(k,1)+car.length/2],...
        [ympc(k,2)-car.width/2, ympc(k,2)+car.width/2, ympc(k,2)+car.width/2, ympc(k,2)-car.width/2], [0 1 0]);
     slope(k)=-(road.centre_rx*cos(car.velocity_angle(k)*Ts/30)*(road.centre_ry^2))/(road.centre_ry*sin(car.velocity_angle(k)*Ts/30)*(road.centre_rx^2));
     angle_turn(k) = atan(slope(k));
     rotate(p, [0 0 1], rad2deg(angle_turn(k)), [ympc(k,1) ympc(k,2) 0]);
    % Obstacles with their safe zones
%     
     for i=1:N
        o(i) = patch([obstacle(i).X(k+1)-obstacle(i).length/2 obstacle(i).X(k+1)-obstacle(i).length/2 obstacle(i).X(k+1)+obstacle(i).length/2 obstacle(i).X(k+1)+obstacle(i).length/2],...
             [obstacle(i).Y(k+1)-obstacle(i).width/2 obstacle(i).Y(k+1)+obstacle(i).width/2 obstacle(i).Y(k+1)+obstacle(i).width/2 obstacle(i).Y(k+1)-obstacle(i).width/2], [1 0 0]);
         
         safe(i) = patch([flSafeX(k+1,i) frSafeX(k+1,i) rlSafeX(k+1,i) rrSafeX(k+1,i)],...
                         [frSafeY(k+1,i) flSafeY(k+1,i) rrSafeY(k+1,i) rlSafeY(k+1,i)],'--');
         safe(i).FaceColor='none';
         safe(i).EdgeColor='[1 0 0]';
         safe(i).LineStyle='--';
         
         coeff(i,k)=-(road.centre_rx*cos(obstacle(i).th+obstacle(i).velocity_angle(k))*(road.centre_ry^2))/(road.centre_ry*sin(obstacle(i).th+obstacle(i).velocity_angle(k))*(road.centre_rx^2));
         angle(i,k) = atan(coeff(i,k));
         rotate(o(i), [0 0 1], rad2deg(angle(i,k)), [obstacle(i).X(k+1) obstacle(i).Y(k+1) 0]);
         rotate(safe(i), [0 0 1], rad2deg(angle(i,k)), [obstacle(i).X(k+1) obstacle(i).Y(k+1) 0]);
     end
% %     
% % %     frame = getframe(gcf);
% % %     writeVideo(writerObj, frame);
% %     
    pause(car.V/length(T))
   delete(refp)
     for i=1:N
         delete(o(i))
         delete(safe(i))
     end
 end
% % % hold off
% % % close(writerObj); % Saves the movie.
% % %close(f);