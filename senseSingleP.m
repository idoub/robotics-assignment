function [out] = senseSingleP(x,y,w,XY1,angle)
    RayX = 200*cos(angle) + x;                                                  % Calculate the ray vector
    RayY = 200*sin(angle) + y;

    XY2 = [x,y,RayX,RayY];                                                      % The scan ray
    result = lineSegmentIntersect(XY1,XY2);

    Dist = zeros(0,1);
    n=0;
    for i=1:length(XY1(:,1))
        if result.intAdjacencyMatrix(i) == 1                                    % If there is a collision
            n=n+1;                                                              % Increment the counter and calculate the distance from the robot to the collision
            Dist(n) = sqrt((result.intMatrixX(i)-x)^2 + (result.intMatrixY(i)-y)^2);
        end
    end

    %intersectX = min(Dist)*cos(w) + x;                                          % Find the point of intersection
    %intersectY = min(Dist)*sin(w) + y;
    %hold on
    %plot([RM.x,intersectX],[RM.y,intersectY],'--r');                            % Plot the ray to the point
    %plot(intersectX,intersectY,'-ro');                                          % Plot the point

    out = min(Dist);
end