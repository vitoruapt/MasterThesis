function [E,F,G,constraintSlope,constraintIntercept] = obstacleComputeCustomConstraint(x,detection,obstacle,laneWidth,lanes,N,m)
%% Compute custom constraints for the obstacle.

carX = x(1);
carY = x(2);

% Compute constraints only if an obstacle is detected. Otherwise, set
% constraint to lower road boundary (the inactive constraint).
% If the street is full of obstacles the ATLASCAR2 must adapt its speed to 
% that of the nearest obstacle; otherwise it can overcome the obstacles.

% Compute the distance between the obstacles
for i=1:N    
    distObs(i)=sqrt((obstacle(m).X(end)-obstacle(i).X(end))^2 +(obstacle(m).Y(end)-obstacle(i).Y(end))^2);
end
maxObs = max(distObs);
n = find(distObs==maxObs);
distObs(m)=[]; % remove the distance of the obstacle m with itself.

if (distObs>30)
    x_max = obstacle(m).X(end)+1000;
    x_min = carX;
    % LEFT OVERTAKING
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
    elseif  ((carX>obstacle(i).rlSafeX) && (carX<=obstacle(i).flSafeX))
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
        E = [0 0; 0 0; 0 0; 0 0; 0 0];
        F = [0 1 0 0;0 -1 0 0;constraintSlope(m) -1 0 0; 1 0 0 0; -1 0 0 0];
        G = [laneWidth*lanes/2; laneWidth*lanes/2;-1*constraintIntercept(m); x_max; -x_min];
    % RIGHT OVERTAKING
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
    elseif  ((carX>obstacle(i).rrSafeX) && (carX<=obstacle(i).frSafeX))
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
        E = [0 0; 0 0; 0 0; 0 0; 0 0];
        F = [0 1 0 0;0 -1 0 0;-1*constraintSlope(m) 1 0 0; 1 0 0 0; -1 0 0 0];
        G = [laneWidth*lanes/2; laneWidth*lanes/2; constraintIntercept(m); x_max; -x_min];
    end
else
    distMaxObs = max(distObs);
    if(distMaxObs>30)
        x_max = obstacle(m).X(end)+1000;
        x_min = carX;
        % OVERTAKING FREE STREET
        % LEFT
        if(obstacle(n).Y(end)>0)
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
    elseif  ((carX>obstacle(i).rlSafeX) && (carX<=obstacle(i).flSafeX))
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
            E = [0 0; 0 0; 0 0; 0 0; 0 0];
            F = [0 1 0 0;0 -1 0 0;constraintSlope(m) -1 0 0; 1 0 0 0; -1 0 0 0];
            G = [laneWidth*lanes/2; laneWidth*lanes/2;-1*constraintIntercept(m); x_max; -x_min];
        % RIGHT
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
    elseif  ((carX>obstacle(i).rrSafeX) && (carX<=obstacle(i).frSafeX))
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
            E = [0 0; 0 0; 0 0; 0 0; 0 0];
            F = [0 1 0 0;0 -1 0 0;-1*constraintSlope(m) 1 0 0; 1 0 0 0; -1 0 0 0];
            G = [laneWidth*lanes/2; laneWidth*lanes/2; constraintIntercept(m); x_max; -x_min];
        end   
    else
        % NO OVERTAKING
        x_max = obstacle(m).X(end)-15;
        x_min = carX;
        if(obstacle(m).Y(end)<=0)
            for i = 1:N
        constraintSlope(i) = 0;
        constraintIntercept(i) = laneWidth*lanes/2;
            end
            E = [0 0; 0 0; 0 0; 0 0; 0 0];
            F = [0 1 0 0;0 -1 0 0;constraintSlope(m) -1 0 0; 1 0 0 0; -1 0 0 0];
            G = [laneWidth*lanes/2; laneWidth*lanes/2;-1*constraintIntercept(m); x_max; -x_min];
        else
            for i = 1:N
        constraintSlope(i) = 0;
        constraintIntercept(i) = -laneWidth*lanes/2;
            end
            E = [0 0; 0 0; 0 0; 0 0; 0 0];
            F = [0 1 0 0;0 -1 0 0;-1*constraintSlope(m) 1 0 0; 1 0 0 0; -1 0 0 0];
            G = [laneWidth*lanes/2; laneWidth*lanes/2; constraintIntercept(m); x_max; -x_min];
        end
    end
end

% %% Define constraint matrices.
% if(obstacle(m).Y(end)<=0)
%    E = [0 0; 0 0; 0 0; 0 0; 0 0];
%    F = [0 1 0 0;0 -1 0 0;constraintSlope(m) -1 0 0; 1 0 0 0; -1 0 0 0];
%    G = [laneWidth*lanes/2; laneWidth*lanes/2;-1*constraintIntercept(m); x_max; -x_min];
% else
%    E = [0 0; 0 0; 0 0; 0 0; 0 0];
%    F = [0 1 0 0;0 -1 0 0;-1*constraintSlope(m) 1 0 0; 1 0 0 0; -1 0 0 0];
%    G = [laneWidth*lanes/2; laneWidth*lanes/2; constraintIntercept(m); x_max; -x_min];
% end
end