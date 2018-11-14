function obstacle = obstacleGenerateObstacleGeometryInfo(obstacle,N)
% Generate obstacle and safe zone geometry.
for i=1:N
   % Obstacle:
% Front left
obstacle(i).flX = obstacle(i).X(end)+obstacle(i).length/2;
obstacle(i).flY = obstacle(i).Y(end)+obstacle(i).width/2;
% Front right
obstacle(i).frX = obstacle(i).X(end)+obstacle(i).length/2;
obstacle(i).frY = obstacle(i).Y(end)-obstacle(i).width/2;
% Rear left
obstacle(i).rlX = obstacle(i).X(end)-obstacle(i).length/2;
obstacle(i).rlY = obstacle(i).flY;
% Rear right
obstacle(i).rrX = obstacle(i).X(end)-obstacle(i).length/2;
obstacle(i).rrY = obstacle(i).frY;

% Safe zone:
% Front left
obstacle(i).flSafeX = obstacle(i).X(end)+obstacle(i).safeDistanceX; 
obstacle(i).flSafeY = obstacle(i).Y(end)+obstacle(i).safeDistanceY;
% Front right
obstacle(i).frSafeX = obstacle(i).X(end)+obstacle(i).safeDistanceX;
obstacle(i).frSafeY = obstacle(i).Y(end)-obstacle(i).safeDistanceY;
% Rear left
obstacle(i).rlSafeX = obstacle(i).X(end)-obstacle(i).safeDistanceX; 
obstacle(i).rlSafeY = obstacle(i).flSafeY;
% Rear right
obstacle(i).rrSafeX = obstacle(i).X(end)-obstacle(i).safeDistanceX;
obstacle(i).rrSafeY = obstacle(i).frSafeY; 
end