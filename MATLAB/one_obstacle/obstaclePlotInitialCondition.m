function f = obstaclePlotInitialCondition(car,obstacle,road)

% Create figure

f = figure;

% Plot the ATLASCAR2

X0 = car.x0(1);
Y0 = car.x0(2);
plot(X0,Y0,'gx'); 
hold on 
grid on
grid minor
p = patch([X0-car.length/2 X0-car.length/2 X0+car.length/2 X0+car.length/2], [Y0-car.width/2, Y0+car.width/2, Y0+car.width/2, Y0-car.width/2], [0 1 0]); 
%rectangle('Position',[X0-car.length/2,Y0 - car.width/2,car.length,car.width],'FaceColor',[0 1 0],'Curvature',0.2,'LineStyle','-','LineWidth', 1.2, 'EdgeColor','k');
axis equal
o = patch([obstacle.X-obstacle.length/2 obstacle.X-obstacle.length/2 obstacle.X+obstacle.length/2 obstacle.X+obstacle.length/2], [obstacle.Y-obstacle.width/2 obstacle.Y+obstacle.width/2 obstacle.Y+obstacle.width/2 obstacle.Y-obstacle.width/2], [1 0 0]);
% Plot the static obstacle (the other car)
% plot(obstacle.X,obstacle.Y,'rx');
%rectangle('Position',[obstacle.rrX,obstacle.rrY,obstacle.length,obstacle.width],'FaceColor',[1 0 0],'Curvature',0.2,'LineStyle','-','LineWidth', 1.2, 'EdgeColor','k');
%rectangle('Position',[obstacle.rrSafeX,obstacle.rrSafeY,...
%    (obstacle.safeDistanceX)*2,(obstacle.safeDistanceY)*2],...
%    'LineStyle','--','EdgeColor','r');

safe = patch([obstacle.flSafeX obstacle.frSafeX obstacle.rlSafeX obstacle.rrSafeX],[obstacle.flSafeY obstacle.frSafeY obstacle.rrSafeY obstacle.rlSafeY],'--');
safe.FaceColor='none';
safe.EdgeColor='[1 0 0]';
safe.LineStyle='--';
%,'EdgeColor','[1 0 0]','LineStyle','--'
delete(p)
delete(o)
delete(safe)
% Plot the safe zone around obstacle.


% Plot the lanes.
X = [X0-car.length;50;90];
Y = [2;2;2];
line(X,Y,'LineStyle','--','Color','k' );
X = [X0-car.length;50;90];
Y = [-2;-2;-2];
line(X,Y,'LineStyle','--','Color','k' );

% Reset the axis.
axis([X0-car.length 90 -road.laneWidth*road.lanes/2 road.laneWidth*road.lanes/2]);
xlabel('X');
ylabel('Y');
title('ATLASCAR2 Moving Obstacle Avoidance');


