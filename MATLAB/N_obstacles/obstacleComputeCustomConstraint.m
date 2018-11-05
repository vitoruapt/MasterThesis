function [E,F,G,constraintSlope,constraintIntercept] = obstacleComputeCustomConstraint(x,detection,obstacle,laneWidth,lanes,N,m)
%% Compute custom constraints for the obstacle.

carX = x(1);
carY = x(2);
% Compute constraints only if an obstacle is detected. Otherwise, set
% constraint to lower road boundary (the inactive constraint).
for i =1:N
    if detection(:,m)
    slope(i) = ( (obstacle(i).rlSafeY - carY)/(obstacle(i).rlSafeX - carX) );
    % If ego car is to the left of the obstacle
    if (carX<=obstacle(i).rlSafeX)
        % if the ego car is already in the adjacent lane, use the safety
        % zone as the constraint.
        if (carY>obstacle(i).rlSafeY)
            constraintSlope(i) = 0;
            constraintIntercept(i) = obstacle(i).rlSafeY;
        else
            % The ego car must be above the line formed from the ego car to
            % safe zone corner for left passing.
            constraintSlope(i) = tan(atan2(slope(i),1));
            constraintIntercept(i) = obstacle(i).rlSafeY - constraintSlope(i)*obstacle(i).rlSafeX;
        end
    % If the ego car is parallel to the obstacle, use the safety zone as
    % the constraint.
    elseif  ((carX>obstacle(i).rlSafeX) && (carX<=obstacle(i).flX))
        constraintSlope(i) = 0;
        constraintIntercept(i) = obstacle(i).rlSafeY; 
    % If the ego car has passed the obstacle, use the inactive constraint
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

%% Define constraint matrices.
E = [0 0;0 0;0 0];
F = [0 1 0 0;0 -1 0 0;constraintSlope(m) -1 0 0];
G = [laneWidth*lanes/2;laneWidth*lanes/2;-1*constraintIntercept(m)];

end