function [detection m]= obstacleDetect(x,obstacle,road,N,car)
% Detect when the vehicle sees an obstacle.

carX = x(1);
carY = x(2);

% Compute the distance between each obstacles and the ATLASCAR2
for i=1:N
    dist2Obstacle(i)= sqrt((obstacle(i).X(end)- carX)^2 + (obstacle(i).Y(end) - carY)^2);    
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

if(sqrt((obstacle(m).X(end)- carX)^2 + (obstacle(m).Y(end) - carY)^2) == obstacle(m).width/2)
    if (m~=N)    
        m=m+1;
    else
        m=m;
    end
end

x_int_car = road.centre_x0 + road.internal_rx*cos(car.angle_init+car.velocity_angle(end));
y_int_car = road.centre_y0 + road.internal_ry*sin(car.angle_init+car.velocity_angle(end));
distCarInternalRadius = sqrt((x_int_car - carX)^2 + (y_int_car - carY)^2); 

for i=1:N
    x_int_obs(i)= road.centre_x0 + road.internal_rx*cos(obstacle(i).th+obstacle(i).velocity_angle(end));
    y_int_obs(i)= road.centre_x0 + road.internal_ry*sin(obstacle(i).th+obstacle(i).velocity_angle(end));
    distObsInternalRadius(i)= sqrt((obstacle(i).X(end) - x_int_obs(i))^2 + (obstacle(i).Y(end) - y_int_obs(i))^2);
    
    flagCloseEnough(i) = (dist2Obstacle(i) < obstacle(i).DetectionDistance);
    flagInLane(i)    = ( abs(distObsInternalRadius(i) - distCarInternalRadius) < 2*road.laneWidth );
    detection(i) =0;%( flagCloseEnough(i) &&  flagInLane(i));
end
end
%%(atan2(carY/carX,1)<atan2(obstacle(i).Y(end)/obstacle(i).X(end),1))