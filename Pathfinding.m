function [Path] = Pathfinding(M,P1,P2)
    %PATHFINDING Finds the shortest path between two points.
    %   Takes a map of type M=[X1,Y1;X2,Y2;...XN-1,YN-1], a start
    %   point P1=[X,Y] and an end point P2=[X,Y] and returns a set of points that
    %   are the shortest path to get from P1 to P2.

    ROBOTWIDTH = 17;
    MINDIST = 2;

    % Define lines from points
    M1 = circshift(M,1);
    Lines = [M1(:,1) - M(:,1), M1(:,2) - M(:,2)];

    % Find normals
    Normals = [Lines(:,2),0-Lines(:,1)];
    Normals = sign(Normals) * ROBOTWIDTH;

    % Add normals to both points at either end of line to make new navigation
    % area
    N1 = circshift(Normals,-1);
    Nav = M + Normals + N1;

    % Make navigation mesh
    Nav1 = circshift(Nav,1);
    NavMesh = [Nav(:,1),Nav(:,2),Nav1(:,1),Nav1(:,2)];
    
    % Check whether the start and end are within the navigation mesh or
    % outside
    [P1in, ~] = inpolygon(P1(1),P1(2),Nav(:,1),Nav(:,2));
    [P2in, ~] = inpolygon(P2(1),P2(2),Nav(:,1),Nav(:,2));
    % If they are outside, find the closest point on the mesh and make that
    % the first point to go to.
    if(P1in == 0)
        [Point1, ~] = ClosestPoint(P1,NavMesh);
    else
        Point1 = P1;
    end
    if(P2in == 0)
        [Point2, ~] = ClosestPoint(P2,NavMesh);
    else
        Point2 = P2;
    end

    % Make a list of nodes for pathfinding
    Nodes = [Point1; Nav; Point2];
    
    Graph = NaN(length(Nodes)^2,4);                                             % Preallocate Graph for speed
    GraphIndex = 1;
    
    % For each node find paths to every other visible node
    for i=1:length(Nodes)
        % Make lines from current node to every node in map
        currentNodeRepeated = repmat(Nodes(i,:),length(Nodes(:,1)),1);
        lines = [Nodes(:,1),Nodes(:,2),currentNodeRepeated(:,1),currentNodeRepeated(:,2)];

        intersections = lineSegmentIntersect(lines,NavMesh);                    % Check for intersections

        % For each line, check it's intersection points and whether it can be
        % added to the graph
        for j=2:length(lines)
            cp = [intersections.intMatrixX(j,:);intersections.intMatrixY(j,:)]';% Make matrix of collision points
            cp(ismember(cp,Nav,'rows'),:) = 0;                                  % Remove actual map points
            cp(all(isnan(cp),2),:) = 0;                                         % Remove NAN rows
            
            % Find center point of each line
            center = [(lines(j,3) - lines(j,1))/2 + lines(j,1),(lines(j,4) - lines(j,2))/2 + lines(j,2)];
            
            [in on] = inpolygon(center(1),center(2),Nav(:,1),Nav(:,2));         % Find whether the center is actually within the navigation mesh
            cp(on)=0;                                                           % Include lines that are incident with navigation mesh edges
            cp(cp==repmat(Nodes(i,:),length(cp(:,1)),1))=0;                     % Drop the collisioin point if the point of collision is the same as the current node

            if(all(cp==0))                                                      % If all elements are zero
                if(in==1)                                                       % If the center does lie within the navigation mesh
                    lineReverse = [lines(j,3:4),lines(j,1:2)];                  % Reverse the line
                    if(~ismember(lineReverse,Graph,'rows'))                     % If the reverse isn't already included in the graph
                        Graph(GraphIndex,:) = lines(j,:);                       % Add this line to the graph
                        GraphIndex = GraphIndex + 1;                            % and increment the index
                    end
                end
            end
        end
    end
    
    Graph = Graph(~isnan(Graph(:,1)),:);                                        % Remove NaN rows from Graph
    Graph(all(Graph(:,1:2)==Graph(:,3:4),2),:) = [];                            % Remove connections from nodes to themselves
    distances = sqrt((Graph(:,1)-Graph(:,3)).^2 + (Graph(:,2)-Graph(:,4)).^2);  % Calculate distances of all lines in graph
    
    % Now just run Dijkstra's algorithm to find the optimal path
    [Paths, NodeDistances] = dijkstra(Graph, Nodes, distances);
    PathIndex = [Paths{length(Paths)},length(Paths)];
    Path = Nodes(PathIndex,:);
    NDist = NodeDistances(PathIndex);
    Path = [Path,NDist];
    Path(NDist < MINDIST,:)=[];                                                 % Drop any node that has a distance less than MINDIST
end