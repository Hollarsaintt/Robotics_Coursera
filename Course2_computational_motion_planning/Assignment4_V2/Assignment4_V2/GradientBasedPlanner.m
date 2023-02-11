function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate

[gx, gy] = gradient (-f);

%%% All of your code should be between the two lines of stars.
% ******************************************************************
r = start_coords(1); c = start_coords(2); % position coordinate initialization.
re = end_coords(1); ce = end_coords(2); %the x and y coordinates of the end configuration.

route = [(start_coords(:))'];

for i = 1:max_its,
    
    if norm([(r - re), (c-ce)]) < 2, % checks if the current position is less than 2 lenghth away from goal
        break
    end
    
    comp = [r c]; % component that will be constantly updated.
    grad = [gx(round(comp(2)), round(comp(1))), gy(round(comp(2)), round(comp(1)))]; % switching is done due to error in the skeleton code
    unit = grad/norm(grad); %only steers in the direction.
    
    vec  = comp + unit; % vec is the new position that is only a unit distance away from initial position comp
    r = vec(1); c = vec(2); %update the position

    route(end+1,:) = [r c]; % add the new position to route.
         
    
end
    



% *******************************************************************
end
