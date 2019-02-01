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

%,'EdgeColor','[1 0 0]','LineStyle','--'
delete(p)

% Reset the axis.
axis([-155 155 -330 330]);
xlabel('X');
ylabel('Y');
title('ATLASCAR2 Moving Obstacle Avoidance');


