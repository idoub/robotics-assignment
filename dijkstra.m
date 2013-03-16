function [ Paths ] = dijkstra( Graph, Nodes, Dist )
%DIJKSTRA Runs Dijkstra's algorithm
%
%   This function runs Dijkstra's algorithm provided a Graph, a list of
%   Nodes, and an array of the distances between the nodes.
%
%   The parameters take the following form:
%   Graph = [:,4] where each row is of the form [X1, Y1, X2, Y2]
%       This defines the connections between each point and has no
%       duplicate connections.
%   Nodes = [:,2] where each row is of the form [X, Y]
%       This defines all the nodes in the graph and has no duplicates
%   Dist = [length(Graph)] is simply a precalculated matrix of the
%       distances defined in Graph

NodeDistances = Inf(length(Nodes),1);
NodeDistances(1) = 0;
i = 1;
Unvisited = (1:length(Nodes))';
Paths = cell(length(Nodes),1,1);

while(~isempty(Unvisited))
    Tentative = Inf(length(Nodes),1);                                           % Initialize new Tentative list of distances for this node
    
    % Find the actual connected nodes to the current
    indxCol1 = ismember(Graph(:,1:2),Nodes(i,:),'rows');                        % Check where this node appears in column 1 of Graph
    indxCol2 = ismember(Graph(:,3:4),Nodes(i,:),'rows');                        % Check where this node appears in column 2 of Graph
    ConnectedNodes = [Graph(indxCol2,1:2);Graph(indxCol1,3:4)];                 % Grab the nodes in the other column to where the current node appears
    nodeIndex = ismember(Nodes,ConnectedNodes,'rows');                          % Generate an index of appearances
    % Connected nodes are now found
    
    distIndex = any([indxCol1,indxCol2],2);                                     % Grab an index of the distances
    Tentative(nodeIndex) = NodeDistances(i) + Dist(distIndex);                  % Calculate a tentative distance
    
    copyIndex = Tentative < NodeDistances;                                      % Create an index of where the tentative distance is less than the current best
    NodeDistances(copyIndex) = Tentative(copyIndex);                            % Copy over only those values
    
    for n=1:length(copyIndex)                                                   % For every value copied over ...
        if(copyIndex(n) == 1)
            Paths{n} = [Paths{i},i];                                            % ... also copy the path to that node
        end
    end
    
    Unvisited(find(Unvisited == i)) = [];                                       % Remove current node from unvisited list
    
     [~, i] = min(Tentative(Unvisited));                                        % Change the new current node to the one with the minimum distance in Unvisited
     i = Unvisited(i);
end

end