function [ closest, distance ] = ClosestPoint(Point, Polygon)
%CLOSESTPOINT Finds the closes point on a polygon to the input point and
%returns the point and the distance
%
%   Point must be provided in the form [X Y]
%   Polygon must be provided in the form [X1, Y1, X2, Y2] for each row in
%   the matrix.

% For each line in the polygon
for i=1:length(Polygon(:,1))
    a = [Polygon(i,1:2) 0] - [Polygon(i,3:4) 0];
    b = [Point 0] - [Polygon(i,3:4) 0];
    r(i) = norm(cross(a,b)) / norm(a);                                          % Distance from the point to the line
end

% The shortest distance between a point and a line forms a right angle with
% the line. Therefore, the distance is the Adjacent to the line
[distance, ind] = min(r);
% Finding the Hypotenuse of the line ...
hypotenuse = pdist([Polygon(ind,1:2);Point])^2;
% ... allows us to find the Opposite
t = sqrt(hypotenuse - (distance^2));
% Then using the vector equation of the line (r = a + tb), we can find the point
direction = Polygon(ind,3:4) - Polygon(ind,1:2);
closest = Polygon(ind,1:2) + t*(direction/norm(direction));

end

