function [detection m]= obstacleDetect(x,obstacle,laneWidth,N)
% Detect when the vehicle sees an obstacle.

carX = x(1);
carY = x(2);

% Compute the distance between each obstacles and the ATLASCAR2
for i=1:N
    dist2Obstacle(i)= sqrt( (obstacle(i).X(end)- carX)^2 + (obstacle(i).Y(end) - carY)^2 );    
end
mindist2Obstacle = min(dist2Obstacle);

% Choose the obstacle closest to the ATLASCAR2  
index=0;
for i=1:length(dist2Obstacle)
    if (dist2Obstacle(i)==mindist2Obstacle)
    index=i;
    end
end
m=index; % represents the index of the closest obstacle 

if(carX>=obstacle(m).flX)
    if (m~=N)    
        m=m+1;
    else
        m=m;
    end
end


for i=1:N
    flagCloseEnough(i) = (dist2Obstacle(i) < obstacle(i).DetectionDistance);
    flagInLane(i)    = ( abs(obstacle(i).Y(end) - carY) < 2*laneWidth );
    detection(i) = ( flagCloseEnough(i) && (carX < obstacle(i).frSafeX) && flagInLane(i));
end
end
        
  




