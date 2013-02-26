function [Path] = Pathfinding(M,P1,P2)
%PATHFINDING Finds the shortest path between two points.
%   Takes a map of type M=[X1,Y1;X2,Y2;...XN-1,YN-1], a start
%   point P1=[X,Y] and an end point P2=[X,Y] and returns a set of points that
%   are the shortest path to get from P1 to P2.

% 1) Generate points from map robot can pass through
%   a) Identify a Maximum, or a Minimum point
%   b) Starting at that point, decrease the clockwise edge by width of
%   robot (should always be moving the edge right if my logic is correct).
%   c) Repeat for all points in series
%   d) Using new edges, identify intersections. These intersections become
%   the vertices of the new 'navigable' map
% 2) Add P1 and P2 to graph of connected points
% 3) Connect all points that can see each other
%   a) Connect each point to every other point, and check intersections
%   with 'navigable' map.
% 4) Find distances between all points in graph.
% 5) Find shortest path from P1 to P2 through graph

% % If the first point in the map is not 0,0, reverse the order of the points
% % Hopefully fixes maps that have been provided in a clockwise direction.
% % The rest of the algorithm assumes that points are provided in an
% % anti-clockwise direction.
% if(~isequal(M(1,:),[0,0]))
%     flipud(M);
% end
% 
% % Split the map into x and y
% MX = M(:,1);
% MY = M(:,2);
% 
% % Define the bounding box
% bounds = [min(MX),min(MY); max(MX),min(MY); max(MX),max(MY); min(MX),max(MY)];
% 
% % Find points on the map that are also part of the bounding box (clearly
% % doesn't work for all maps) and return their index.
% boundingPointIndex = ismember(M,bounds,'rows')
% 
% 
% % Starting with the first line from the bounding box, move it towards the
% % end point.

ROBOTWIDTH = 100;

hold on
plot(M(:,1),M(:,2))

% Define lines from points
M1 = circshift(M,1);
V1 = (M1(:,1:2) - M(:,1:2));
V2 = (M(:,1:2) - M1(:,1:2));
V2 = circshift(V2,-1);

V1 = V1/norm(V1);
V2 = V2/norm(V2);

bi = (V1 + V2) * ROBOTWIDTH;
bi = bi + M;
nan = NaN(size(M));
bisectors = reshape([M(:) bi(:) nan(:)]',3*size(M,1), []);
plot(bisectors(:,1),bisectors(:,2),'r');

[xi,yi] = polyxpoly(bisectors(:,1),bisectors(:,2),bisectors(:,1),bisectors(:,2))
mapshow(xi,yi,'DisplayType','point','Marker','o')
end

