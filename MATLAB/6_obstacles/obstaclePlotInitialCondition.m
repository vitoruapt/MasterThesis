function f = obstaclePlotInitialCondition(car,obstacle,road,N)

% Create figure

f = figure ('Position',[100, 100, 700, 110], 'PaperPositionMode','auto');


% Starting position of the ATLASCAR2 ('green cross')
X0 = car.x0(1);
Y0 = car.x0(2);
plot(X0,Y0,'gx');
hold on 
grid on
grid minor
axis equal

% ATLASCAR2 initialization
p = patch([X0-car.length/2 X0-car.length/2 X0+car.length/2 X0+car.length/2], [Y0-car.width/2, Y0+car.width/2, Y0+car.width/2, Y0-car.width/2], [0 1 0]); 

% Obstacles initialization with their safe zones
for i=1:N
    o(i) = patch([obstacle(i).X-obstacle(i).length/2 obstacle(i).X-obstacle(i).length/2 obstacle(i).X+obstacle(i).length/2 obstacle(i).X+obstacle(i).length/2], [obstacle(i).Y-obstacle(i).width/2 obstacle(i).Y+obstacle(i).width/2 obstacle(i).Y+obstacle(i).width/2 obstacle(i).Y-obstacle(i).width/2], [1 0 0]);
    safe(i) = patch([obstacle(i).flSafeX obstacle(i).frSafeX obstacle(i).rlSafeX obstacle(i).rrSafeX],[obstacle(i).flSafeY obstacle(i).frSafeY obstacle(i).rrSafeY obstacle(i).rlSafeY],'--');
    safe(i).FaceColor='none';
    safe(i).EdgeColor='[1 0 0]';
    safe(i).LineStyle='--';
end

% For the animation remove the initializations
delete(p)
    for i=1:N
        delete(o(i))
        delete(safe(i))
    end

% Plot the lanes.
X = [X0-car.length;50;obstacle(end).X+obstacle(end).length];
Y = [2;2;2];
line(X,Y,'LineStyle','--','Color','k');
X = [X0-car.length;50;obstacle(end).X+obstacle(end).length];
Y = [-2;-2;-2];
line(X,Y,'LineStyle','--','Color','k');

% Reset the axis.
axis([X0-car.length 120 -road.laneWidth*road.lanes/2 road.laneWidth*road.lanes/2]);
xlabel('X');
ylabel('Y');
%title('ATLASCAR2 Moving Obstacle Avoidance');
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
pos = get(f,'Position');
set(f,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos(3), pos(4)])
