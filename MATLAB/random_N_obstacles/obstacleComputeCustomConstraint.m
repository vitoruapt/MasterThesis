function [E,F,G,constraintSlope,constraintIntercept] = obstacleComputeCustomConstraint(x,detection,obstacle,laneWidth,lanes,N,m)
%% Compute custom constraints for the obstacle.

carX = x(1);
carY = x(2);
% Compute constraints only if an obstacle is detected. Otherwise, set
% constraint to lower road boundary (the inactive constraint).
if(obstacle(m).Y(end)<=0)
   for i=1:N 
   slope(i) = ( (obstacle(i).rlSafeY - carY)/(obstacle(i).rlSafeX - carX) );    
   pop(i) = ( (obstacle(i).rrSafeY - carY)/(obstacle(i).rrSafeX - carX) );
   if detection(:,m)
    % If the ATLASCAR2 is to the left of the obstacle
    if (carX<=obstacle(i).rlSafeX)
        % if the ATLASCAR2 is already in the adjacent lane, use the safety
        % zone as the constraint.
        if (carY>obstacle(i).rlSafeY)
            constraintSlope(i) = 0;
            constraintIntercept(i) = obstacle(i).rlSafeY;
        else
            % The ATLASCAR2 must be above the line formed from the ATLASCAR2 to
            % safe zone corner for left passing.
            constraintSlope(i) = tan(atan2(slope(i),1));
            constraintIntercept(i) = obstacle(i).rlSafeY - constraintSlope(i)*obstacle(i).rlSafeX;
        end
    % If the ATLASCAR2 is parallel to the obstacle, use the safety zone as
    % the constraint.
    elseif  ((carX>obstacle(i).rlSafeX) && (carX<=obstacle(i).flX))
        constraintSlope(i) = 0;
        constraintIntercept(i) = obstacle(i).rlSafeY; 
    % If the ATLASCAR2 has passed the obstacle, use the inactive constraint
    % to go back to the center lane.
    else 
        constraintSlope(i) = 0;
        constraintIntercept(i) = obstacle(i).rlSafeY; %-laneWidth*lanes/2;
    end
    % If there is another obstacles in the detection zone se the safety zone as
    % the constraint. Otherwise use the inactive constraint
    % to go back to the center lane.
    elseif(detection(:,:)==zeros(1,N))
        constraintSlope(i) = 0;
        constraintIntercept(i) = -laneWidth*lanes/2;
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
    if (carX<=obstacle(i).rrSafeX)
        % if the ATLASCAR2 is already in the adjacent lane, use the safety
        % zone as the constraint.
        if (carY<obstacle(i).rrSafeY)
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
    elseif  ((carX>obstacle(i).rrSafeX) && (carX<=obstacle(i).frX))
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
        constraintIntercept(i) = laneWidth*lanes/2;
    else
        constraintSlope(i) = 0;
        constraintIntercept(i) = obstacle(i).rrSafeY;
    end
end
end
%% Define constraint matrices.
if(obstacle(m).Y(end)<=0)
   E = [0 0;0 0;0 0];
   F = [0 1 0 0;0 -1 0 0;constraintSlope(m) -1 0 0];
   G = [laneWidth*lanes/2;laneWidth*lanes/2;-1*constraintIntercept(m)];
else
    E = [0 0;0 0;0 0];
    F = [0 1 0 0;0 -1 0 0;-1*constraintSlope(m) 1 0 0];
    G = [laneWidth*lanes/2;laneWidth*lanes/2;constraintIntercept(m)];
end
end