function [ output_args ] = dijkstra( Graph, Nodes, Dist )
%DIJKSTRA Summary of this function goes here
%   Detailed explanation goes here


Tentative = Inf(length(Nodes),1);
NodeDistances = Inf(length(Nodes),1);
NodeDistances(1) = 0;
i = 1;
Unvisited = (1:length(Nodes))';
Visited = [];

%while length(Unvisited) > 0
for j=1:2%length(Nodes)
    % Find the actual connected nodes to the current
    indxCol1 = ismember(Graph(:,1:2),Nodes(i,:),'rows');                        % Check where this node appears in column 1 of Graph
    indxCol2 = ismember(Graph(:,3:4),Nodes(i,:),'rows');                        % Check where this node appears in column 2 of Graph
    ConnectedNodes = [Graph(indxCol2,1:2);Graph(indxCol1,3:4)];                 % Grab the nodes in the other column to where the current node appears
    nodeIndex = ismember(Nodes,ConnectedNodes,'rows');                          % Generate an index of appearances
    
    distIndex = any([indxCol1,indxCol2],2);                                     % Grab an index of the distances
    Tentative(nodeIndex) = NodeDistances(i) + Dist(distIndex);                  % Calculate a tentative distance
    
    copyIndex = Tentative < NodeDistances;                                      % Create an index of where the tentative distance is less than the current best
    NodeDistances(copyIndex) = Tentative(copyIndex);                            % Copy over only those values
    
    Visited = [Visited;i]                                                      % Add current node to visited list
    Unvisited(i,:) = []                                                        % Remove current node from unvisited list
    
    Tentative(Unvisited)
    
    [~, i] = min(Tentative(Unvisited))
    NodeDistances(i)
end

    % Dijkstra's algorithm
%      1  function Dijkstra(Graph, source):
%      2      for each vertex v in Graph:                                // Initializations
%      3          dist[v] := infinity ;                                  // Unknown distance function from 
%      4                                                                 // source to v
%      5          previous[v] := undefined ;                             // Previous node in optimal path
%      6      end for                                                    // from source
%      7      
%      8      dist[source] := 0 ;                                        // Distance from source to source
%      9      Q := the set of all nodes in Graph ;                       // All nodes in the graph are
%     10                                                                 // unoptimized - thus are in Q
%     11      while Q is not empty:                                      // The main loop
%     12          u := vertex in Q with smallest distance in dist[] ;    // Start node in first case
%     13          remove u from Q ;
%     14          if dist[u] = infinity:
%     15              break ;                                            // all remaining vertices are
%     16          end if                                                 // inaccessible from source
%     17          
%     18          for each neighbor v of u:                              // where v has not yet been 
%     19                                                                 // removed from Q.
%     20              alt := dist[u] + dist_between(u, v) ;
%     21              if alt < dist[v]:                                  // Relax (u,v,a)
%     22                  dist[v] := alt ;
%     23                  previous[v] := u ;
%     24                  decrease-key v in Q;                           // Reorder v in the Queue
%     25              end if
%     26          end for
%     27      end while
%     28  return dist;


end

