function [E,F,G,constraintSlope,constraint_x, constraint_y] = obstacleComputeCustomConstraint(x,detection,obstacle,laneWidth,lanes,N,m,road,car)
%% Compute custom constraints for the obstacle.

carX = x(1);
carY = x(2);
Ts=0.02;
slopeangle = -(road.centre_rx*cos(car.angle_init+car.velocity_angle(end))*(road.centre_ry^2))/(road.centre_ry*sin(car.angle_init+car.velocity_angle(end))*(road.centre_rx^2));
delta =  atan2(slopeangle,1);


constraint_x_cen = road.centre_x0 + road.centre_rx*cos(car.velocity_angle(end)*Ts/35);
constraint_y_cen = road.centre_x0 + road.centre_rx*cos(car.velocity_angle(end)*Ts/35);

t = linspace(1,360,361); 
constraint_x_iniz_int = -(road.centre_x0 + road.internal_rx*cos(t));
constraint_y_iniz_int = -(road.centre_y0 + road.internal_ry*sin(t));

constraint_x_iniz_ext = road.centre_x0 + road.external_rx*cos(t);
constraint_y_iniz_ext = road.centre_y0 + road.external_ry*sin(t);

constraint_x = [constraint_x_iniz_int, constraint_x_iniz_ext];
constraint_y = [constraint_y_iniz_int, constraint_y_iniz_ext];
for i=1:N
    x_int_obs(i)= road.centre_x0 + road.internal_rx*cos(obstacle(i).th+obstacle(i).velocity_angle(end));
    y_int_obs(i)= road.centre_x0 + road.internal_ry*sin(obstacle(i).th+obstacle(i).velocity_angle(end));
end
% Compute constraints only if an obstacle is detected. Otherwise, set
% constraint to lower road boundary (the inactive constraint).
if(sqrt((obstacle(m).X(end) - x_int_obs(m))^2 + (obstacle(m).Y(end) - y_int_obs(m))^2)<=(road.width/2))
   for i=1:N 
   slope(i) = ( (obstacle(i).rlSafeY - carY)/(obstacle(i).rlSafeX - carX) );    
   pop(i) = ( (obstacle(i).rrSafeY - carY)/(obstacle(i).rrSafeX - carX) );
   if detection(:,m)
    % If the ATLASCAR2 is to the left of the obstacle
    if (sqrt((obstacle(i).X(end) - carX)^2 + (obstacle(i).Y(end) - carY)^2)>=sqrt((obstacle(i).X(end) - obstacle(i).rlSafeX)^2 + (obstacle(i).Y(end) - obstacle(i).rlSafeY)^2)/2)
        % if the ATLASCAR2 is already in the adjacent lane, use the safety
        % zone as the constraint.
    if (sqrt((road.centre_x0 - carX)^2 + (road.centre_y0 - carY)^2)>=sqrt((road.centre_x0 - obstacle(i).rlSafeX)^2 + (road.centre_y0 - obstacle(i).rlSafeY)^2))
            constraintSlope(i) = 0;
            constraintIntercept(i)= obstacle(i).rlSafeY;
    else
            % The ATLASCAR2 must be above the line formed from the ATLASCAR2 to
            % safe zone corner for left passing.
            constraintSlope(i) = tan(atan2(slope(i),1));
            constraintIntercept(i) = obstacle(i).rlSafeY - constraintSlope(i)*obstacle(i).rlSafeX;
        end
    % If the ATLASCAR2 is parallel to the obstacle, use the safety zone as
    % the constraint.
    elseif  (sqrt((obstacle(i).X(end) - carX)^2 + (obstacle(i).Y(end) - carY)^2)>=sqrt((obstacle(i).X(end) - obstacle(i).rlSafeX)^2 + (obstacle(i).Y(end) - obstacle(i).rlSafeY)^2)/2)
        constraintSlope(i) = 0;
        constraintIntercept(i) = obstacle(i).rlSafeY;
    % If the ATLASCAR2 has passed the obstacle, use the inactive constraint
    % to go back to the center lane.
    else 
        constraintSlope(i) = 0;
        constraintIntercept(i) = obstacle(i).rlSafeY;
    end
    % If there is another obstacles in the detection zone se the safety zone as
    % the constraint. Otherwise use the inactive constraint
    % to go back to the center lane.
    elseif(detection(:,:)==zeros(1,N))
        constraintSlope(i) = 0;
        constraintIntercept_y(i) = road.centre_y0+road.centre_ry*sin(car.velocity_angle(end)*Ts/35);
        constraintIntercept_x(i) = road.centre_x0+road.centre_rx*cos(car.velocity_angle(end)*Ts/35);
   else
        constraintSlope(i) = 0;
        constraintIntercept(i) = obstacle(i).rlSafeY;
   end
end
else
for i=1:N
   slope(i) = ( (obstacle(i).rlSafeY - carY)/(obstacle(i).rlSafeX - carX) );    
   pop(i) = ( (obstacle(i).rrSafeY - carY)/(obstacle(i).rrSafeX - carX) );
   if detection(:,m)
    % If the ATLASCAR2 is to the right of the obstacle
   if (sqrt((obstacle(i).X(end) - carX)^2 + (obstacle(i).Y(end) - carY)^2)>=sqrt((obstacle(i).X(end) - obstacle(i).rrSafeX)^2 + (obstacle(i).Y(end) - obstacle(i).rrSafeY)^2))
        % if the ATLASCAR2 is already in the adjacent lane, use the safety
        % zone as the constraint.
    if (sqrt((road.centre_x0 - carX)^2 + (road.centre_y0 - carY)^2)>=sqrt((road.centre_x0 - obstacle(i).rrSafeX)^2 + (road.centre_y0 - obstacle(i).rrSafeY)^2))
            constraintSlope(i) = 0;
            constraintIntercept(i) = obstacle(i).rrSafeY;
        else
            % The ATLASCAR2 must be below the line formed from the ATLASCAR2 to
            % safe zone corner for right passing.
            constraintSlope(i) = tan(atan2(pop(i),1));
            constraintIntercept(i) = obstacle(i).rrSafeY - constraintSlope(i)*obstacle(i).rrSafeX;
        end
    % If the ATLASCAR2 is parallel to the obstacle, use the safety zone as
    % the constraint.
    elseif  (sqrt((obstacle(i).X(end) - carX)^2 + (obstacle(i).Y(end) - carY)^2)<sqrt((obstacle(i).X(end) - obstacle(i).rrSafeX)^2 + (obstacle(i).Y(end) - obstacle(i).rrSafeY)^2))
        constraintSlope(i) = 0;
        constraintIntercept(i) = obstacle(i).rrSafeY; 
        
    % If the ATLASCAR2 has passed the obstacle, use the inactive constraint
    % to go back to the center lane.
    else 
        constraintSlope(i) = 0;
        constraintIntercept(i) = obstacle(i).rrSafeY; %-laneWidth*lanes/2;
    end
    % If there is another obstacles in the detection zone se the safety zone as
    % the constraint. Otherwise use the inactive constraint
    % to go back to the center lane.
    elseif(detection(:,:)==zeros(1,N))
        constraintSlope(i) = 0;
        constraintIntercept(i) = constraint_y(1);
    else
        constraintSlope(i) = 0;
        constraintIntercept(i) = obstacle(i).rrSafeY;
    end
end
end
%% Define constraint matrices.
if(sqrt((obstacle(m).X(end) - x_int_obs(m))^2 + (obstacle(m).Y(end) - y_int_obs(m))^2)<=(road.width/2))
   E = [0 0; 0 0; 0 0; 0 0; 0 0; 0 0];
   F = [0 1 0 0; 0 -1 0 0; constraintSlope(m) 1 0 0 ;1 0 0 0; -1 0 0 0;1 constraintSlope(m) 0 0];
   G = [constraint_y(2);-constraint_y(1);-1*constraintIntercept_y(m);constraint_x(2);-constraint_x(1); -1*constraintIntercept_x(m)];
else
   E = [0 0; 0 0; 0 0; 0 0; 0 0];
   F = [0 1 0 0;0 -1 0 0;-1*constraintSlope(m) 1 0 0; 1 0 0 0;-1 0 0 0];
   G = [constraint_y(2);-constraint_y(1);constraintIntercept(m); constraint_x(2);-constraint_x(1)];
end
end