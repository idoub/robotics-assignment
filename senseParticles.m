function [out] = senseParticles(x,y,w,M,numreadings,angleError)
    M1 = circshift(M,1);                                                        % Shift M circularly by one
    XY1 = [M(:,1),M(:,2),M1(:,1),M1(:,2)];                                      % Form new matrix of each element in M with the next element

    angleChange = (2*pi)/numreadings;                                           % Calculate the change in angle according to the number of readings requested
    angle = w;% -pi;                                                            % Starting angle is -pi (relative to robot this would be backwards)
    out = zeros(1,numreadings);                                                 % Change this to w to get the first measure forward the robot (Antoine 22/02/13)
    for m=1:numreadings                                                         % For each reading
        out(m) = senseSingleP(x,y,w,XY1,angle+angleError(m));                                 % Take a sensor measurement
        angle = angle + angleChange;                                            % Adjust the angle of the next measurement
    end

    %figure(2);
    %hold on;
    %plot(0:numreadings-1,out,'*');
    %coefs = polyfit(0:numreadings-1,out,numreadings/2);                         % These two linesplot a polynomial of degree numreadings/2that best fits the point already on the graph
    %plot(0:0.1:numreadings-1,polyval(coefs,0:0.1:numreadings-1),'-r');
end
        
       