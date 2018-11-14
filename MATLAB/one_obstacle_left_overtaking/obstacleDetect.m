function detection = obstacleDetect(x,obstacle,laneWidth)
% Detect when the vehicle sees an obstacle.

carX = x(1);
carY = x(2);
dist2Obstacle   = sqrt( (obstacle.X(end)- carX)^2 + (obstacle.Y - carY)^2 );
flagCloseEnough = (dist2Obstacle < obstacle.DetectionDistance);
flagInLane      = ( abs(obstacle.Y - carY) < 2*laneWidth );
detection = ( flagCloseEnough && (carX < obstacle.frSafeX) && flagInLane );