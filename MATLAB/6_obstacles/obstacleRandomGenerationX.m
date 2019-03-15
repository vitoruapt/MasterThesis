function obstacle_random_X = obstacleRandomGenerationX(length,distance,N)

% This function creates random positions in the x coordinate for the obstacles 
% where the parameters are the length of the street, the minimum distance between two
% consecutive obstacles and the number of the cars. 

E=length-(N-1)*distance;  % excess space for points

% generate N+1 random values; 

Ro=rand(N+1,1);     % random vector

% normalize so that the extra space is consumed
% extra value is the amount of extra space "unused"

Rn=E*Ro(1:N)/sum(Ro); % normalize

% spacing of points

S=distance*ones(N,1)+Rn;  

% location of points, adjusted to "start" at 0
obstacle_random_X=cumsum(S)-1;

end