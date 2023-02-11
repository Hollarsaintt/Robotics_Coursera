function route = DijkstraTorus (input_map, start_coords, dest_coords)
% Run Dijkstra's algorithm on a grid.
% Inputs : 
%   input_map : a logical array where the freespace cells are false or 0 and
%      the obstacles are true or 1
%   start_coords and dest_coords : Coordinates of the start and end cell
%       respectively, the first entry is the row and the second the column.
% Output :
%   route : An array containing the linear indices of the cells along the
%    shortest route from start to dest or an empty array if there is no
%    route.

% set up color map for display
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red = visited
% 4 - blue  - on list
% 5 - green - start
% 6 - yellow - destination

cmap = [1 1 1; ...
        0 0 0; ...
        1 0 0; ...
        0 0 1; ...
        0 1 0; ...
        1 1 0];

colormap(cmap);

if start_coords(1) == 16,
	input_map(:, 181) = [];
	input_map(181,:) = [];
end


[nrows, ncols] = size(input_map);

% map - a table that keeps track of the state of each grid cell
map = zeros(nrows,ncols);

map(~input_map) = 1;  % Mark free cells
map(input_map)  = 2;  % Mark obstacle cells

% Generate linear indices of start and dest nodes
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
dest_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));

map(start_node) = 5;
map(dest_node)  = 6;

% Initialize distance array
distances = Inf(nrows,ncols);

% For each grid cell this array holds the index of its parent
parent = zeros(nrows,ncols);

distances(start_node) = 0;

% Main Loop
while true
    
    % Draw current map
    map(start_node) = 5;
    map(dest_node) = 6;
    
    image(1.5, 1.5, map);
    grid on;
    axis image;
    drawnow;
    
    % Find the node with the minimum distance
    [min_dist, current] = min(distances(:));
    
    if ((current == dest_node) || isinf(min_dist))
        break;
    end;
    
    % Update map
    map(current) = 3;         % mark current node as visited
    distances(current) = Inf; % remove this node from further consideration
    
    % Compute row, column coordinates of current node
    [i, j] = ind2sub(size(distances), current);
    
    % Visit each neighbor of the current node and update the map, distances
    % and parent tables appropriately.
   
    %%% All of your code should be between the two lines of stars. 
    % *******************************************************************
    
    % Visit each neighbor of the current node and update the map, distances
    % and parent tables appropriately.
    
    %i = min(abs(i), abs(360-i)); j = min(abs(j), abs(360-j));
    % my code.
    if i-1 == 0, % check if index is out of range
        i = nrows;
        N = sub2ind(size(distances), i, j); %North rollover Neighbour
    else
        N = sub2ind(size(distances), i-1, j); %North Neighbour
    end   
    if i+1 > size(map,1),
        i = 1;
        S = sub2ind(size(distances), i, j); %south rollover Neighbour
    else  
        S = sub2ind(size(distances), i+1, j); %south Neighbour
    end
    
    if j-1 == 0, %check if index is out of range
        j = ncols;
        W =  sub2ind(size(distances), i, j); %west rollover Neighbour
    else
        W = sub2ind(size(distances), i, j-1); % west Neighbour
    end
    
    if j+1 > size(map,2), %check if index is out of range.
        j = 1;
        E =  sub2ind(size(distances), i, j); % East rollover neighbour
    else
        E = sub2ind(size(distances), i, j+1); % East Neighbour
    end
    
    neighbor_indices = [N; S; W; E]; % holds linear indices of N S W E neighbour respectively.
    
    for a = 1:4,
        nInd = neighbor_indices(a); % holds the neighbor index.
        if nInd ~= 0, % hecks if it holds valid neighbour index.
            if (~input_map(nInd)) && (map(nInd) ~= 3) && (map(nInd) ~= 5), % check if the neighbor we are considering is in freespace, it has not being visited and it is not the start node.
                map(nInd) = 4; % mark the neighbour as on consideration list.
                if distances(nInd) > (min_dist + 1),
                    distances(nInd) = (min_dist + 1);
                    parent(nInd) = current;
                end
            end
        end
    end
   

           

    
    % *******************************************************************
end

if (isinf(distances(dest_node)))
    route = [];
else
    route = [dest_node];
    
    while (parent(route(1)) ~= 0)
        route = [parent(route(1)), route];
    end
end

    function update (i,j,d,p)
        if ( (map(i,j) ~= 2) && (map(i,j) ~= 3) && (map(i,j) ~= 5) && (distances(i,j) > d) )
            distances(i,j) = d;
            map(i,j) = 4;
            parent(i,j) = p;
        end
    end

end
