function obstacle = obstacleGenerateObstacleGeometryInfo(obstacle)
% Generate obstacle and safe zone geometry.
 
% Obstacle:
% Front left
obstacle.flX = obstacle.X(end)+obstacle.length/2;
obstacle.flY = obstacle.Y+obstacle.width/2;
% Front right
obstacle.frX = obstacle.X(end)+obstacle.length/2;
obstacle.frY = obstacle.Y-obstacle.width/2;
% Rear left
obstacle.rlX = obstacle.X(end)-obstacle.length/2;
obstacle.rlY = obstacle.flY;
% Rear right
obstacle.rrX = obstacle.X(end)-obstacle.length/2;
obstacle.rrY = obstacle.frY;

% Safe zone:
% Front left
obstacle.flSafeX = obstacle.X(end)+obstacle.safeDistanceX; 
obstacle.flSafeY = obstacle.Y+obstacle.safeDistanceY;
% Front right
obstacle.frSafeX = obstacle.X(end)+obstacle.safeDistanceX;
obstacle.frSafeY = obstacle.Y-obstacle.safeDistanceY;
% Rear left
obstacle.rlSafeX = obstacle.X(end)-obstacle.safeDistanceX; 
obstacle.rlSafeY = obstacle.flSafeY;
% Rear right
obstacle.rrSafeX = obstacle.X(end)-obstacle.safeDistanceX;
obstacle.rrSafeY = obstacle.frSafeY;