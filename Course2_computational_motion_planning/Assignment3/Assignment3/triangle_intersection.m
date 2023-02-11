function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************

flag = true;
for e = 1:6,
   val = checkC(e, P1, P2);
   if val == true,
       flag = false;
       break
   end
end

function booll = checkC(e,P1,P2) % returns true or false true if edge e is a separating edge and false otherwise.
        booll = false; %default value.
        Em = [ (P1(1,2) - P1(2,2)) / (P1(1,1) - P1(2,1));
               (P1(1,2) - P1(3,2)) / (P1(1,1) - P1(3,1));
               (P1(2,2) - P1(3,2)) / (P1(2,1) - P1(3,1));
               (P2(1,2) - P2(2,2)) / (P2(1,1) - P2(2,1));
               (P2(1,2) - P2(3,2)) / (P2(1,1) - P2(3,1));
               (P2(2,2) - P2(3,2)) / (P2(2,1) - P2(3,1));
               ]; % holds the gradient of all the six edges. that is change in y / change in x.
           
        last_vertex = [P1(3,:); P1(2,:); P1(1,:); P2(3,:); P2(2,:); P2(1,:)]; %holds the last vertex corresponding to each edges e.
        
        c_edge = [P1(1,2) - (Em(1)*(P1(1,1)));
                    P1(1,2) - (Em(2) * (P1(1,1)));
                    P1(2,2) - (Em(3) * (P1(2,1)));
                    P2(1,2) - (Em(4) * (P2(1,1)));
                    P2(1,2) - (Em(5) * (P2(1,1)));
                    P2(2,2) - (Em(6) * (P2(2,1)));
                    ]; % finds the intercept of all the edges.
                    
        
        if Em(e) == 0, % corresponds to c equal to the y value at any time.
            m = 0; j = 2;
            booll = cSign(e, j, last_vertex, m, P1,P2,c_edge);
             
        elseif Em(e) == inf, %corresponds to c equal to the x value at any time
            m = 0; j = 1;
            booll = cSign(e, j, last_vertex, m, P1, P2,c_edge);
        else
            m = Em(e); j = 2;
            booll = cSign(e, j, last_vertex, m, P1, P2,c_edge);
        end
           
    
    function booll = cSign(e, j, last_vertex, m, P1, P2, c_edge) % returns boolean true if c is in different side otherwise returns false.
        booll = false; %default return value.
        c = last_vertex(e,j)- (m*last_vertex(e,1)); % the third vertex y point.
        
        if ismember(e, [4,5,6]), % if the edge is part of the 2nd point switch the point to satisfy the calculation.
            [P2 , ~] = deal(P1,P2); % swaps p1 and p2.
        end
        for i = 1:3,
            c_2 = P2(i,j) - (m*P2(i,1));
            ctrl = true; %default value.
            % checks if the intercepts of c and c_2 are on the same side,
            % if they are then the edge e corresponding to that scenario is
            % not a separating edge.
            if (c > c_edge(e) && c_2 > c_edge(e)) || (c < c_edge(e) && c_2 < c_edge(e)) || (c_2 == c_edge(e)),
                 ctrl = false; %corresponds to not opposite. 
                 break
            end
        end
        if ctrl == true, % corresponds to opposite c value
            booll = true; %means a e is a seperating edge for the two triangles.
        end
    end
        
        
end
    
% *******************************************************************
end