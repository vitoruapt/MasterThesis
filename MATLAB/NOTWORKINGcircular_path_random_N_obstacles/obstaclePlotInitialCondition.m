function f = obstaclePlotInitialCondition(car,obstacle,road,N)

% Create figure

f = figure;

% Starting position of the ATLASCAR2 ('green cross')
X0 = car.x0(1);
Y0 = car.x0(2);
plot(X0,Y0,'gx');
hold on 
grid on
grid minor
axis equal

% Plot the lanes.

t=0;
for i=1:road.lanes+1
e(i) = ellipse(road.centre_x0,road.centre_y0,road.external_rx+road.laneWidth-road.laneWidth*i,road.external_ry+road.laneWidth-road.laneWidth*i);
    if (i>1 && i<road.lanes+1)
        e(i).LineStyle='--';
    else
        e(i).LineStyle='-';
    end
end
p = patch([car.x_init-car.length/2, car.x_init-car.length/2, car.x_init+car.length/2, car.x_init+car.length/2],...
          [car.y_init-car.width/2, car.y_init+car.width/2, car.y_init+car.width/2, car.y_init-car.width/2], [0 1 0]);
    rotate(p ,[0,0,1], rad2deg(car.u0(2)), [car.x_init car.y_init 0]);

for i=1:N
    o(i) = patch ([obstacle(i).X-obstacle(i).length/2, obstacle(i).X-obstacle(i).length/2, obstacle(i).X+obstacle(i).length/2, obstacle(i).X+obstacle(i).length/2],...
                  [obstacle(i).Y-obstacle(i).width/2, obstacle(i).Y+obstacle(i).width/2, obstacle(i).Y+obstacle(i).width/2, obstacle(i).Y-obstacle(i).width/2], [1 0 0]);
    safe(i) = patch([obstacle(i).flSafeX obstacle(i).frSafeX obstacle(i).rlSafeX obstacle(i).rrSafeX],...
                    [obstacle(i).flSafeY obstacle(i).frSafeY obstacle(i).rrSafeY obstacle(i).rlSafeY],'--');
    safe(i).FaceColor='none';
    safe(i).EdgeColor='[1 0 0]';
    safe(i).LineStyle='--';

    coeff(i)=-(road.centre_rx*cos(obstacle(i).th)*(road.centre_ry^2))/(road.centre_ry*sin(obstacle(i).th)*(road.centre_rx^2));
    angle(i) = atan2(coeff(i),1);
    rotate(o(i), [0 0 1], rad2deg(angle(i)), [obstacle(i).X obstacle(i).Y 0]);
    rotate(safe(i), [0 0 1], rad2deg(angle(i)), [obstacle(i).X obstacle(i).Y 0]);
end

% For the animation remove the initializations
%delete(p)
%     for i=1:N
%         delete(o(i))
%         delete(safe(i))
%     end

xlabel('X');
ylabel('Y');
title('ATLASCAR2 Moving Obstacle Avoidance');


