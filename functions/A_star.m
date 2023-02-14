function xy = A_star(MAP_IMG,INIT_POS,GOAL)

MAP = robotics.OccupancyGrid(MAP_IMG, 25);
MAP.inflate(0.07);

map_gridsize = MAP.GridSize;
h = map_gridsize(1);
w = map_gridsize(2);

start = [round(INIT_POS(1)*MAP.GridSize(2)/MAP.XWorldLimits(2)), round(INIT_POS(2)*MAP.GridSize(1)/MAP.YWorldLimits(2))];
parent = [start(1), start(2)];
goal = [round(GOAL(1)*MAP.GridSize(2)/MAP.XWorldLimits(2)), round(GOAL(2)*MAP.GridSize(1)/MAP.YWorldLimits(2))];

costs = ones(w,h)*inf;% cost values for each cell, filled incrementally. Initialize with infinity
costs(parent(1),parent(2)) = 0;
heuristics = zeros(w,h);% estimated costs to the goal.
closed_list = zeros(w,h);% cells that have been visited

% these matrices implicitly store the path
% by containing the x and y position of the previous
% node, respectively. Following these starting at the goal 
% until -1 is reached returns the computed path, see at the bottom
previous_x = zeros(w,h)-1;
previous_y = zeros(w,h)-1;

while (parent(1) ~= goal(1) || parent(2) ~= goal(2))

  %generate mask to assign infinite costs for cells already visited
  closed_mask = closed_list;
  closed_mask( closed_mask == 1 ) = Inf; 
  
  %find the candidates for expansion (open list/frontier)
  open_list = costs + closed_mask + heuristics;
            
  %check if a non-infinite entry exists in open list (list is not empty)
  if min(open_list(:)) == Inf
    disp 'no valid path found';
    break
  end
  
  %find the cell with the minimum cost in the open list
  [x,y] = find(open_list == min(open_list(:)));
  parent_y = y(1);
  parent_x = x(1);
  parent = [parent_x, parent_y];
  
  %put parent in closed list
  closed_list(parent_x,parent_y) = 1;
    
  %get neighbors of parent
  n = neighbors(parent, [w,h]);
  for i = 1:size(n,1)
    child_y = n(i,2);
    child_x = n(i,1);
    child = [child_x, child_y];
    
    %calculate the cost of reaching the cell
    cost_val = costs(parent_x,parent_y) + edge_cost(parent, child, MAP);
      
    %Exercise 2: estimate the remaining costs from the cell to the goal
    heuristic_val = heuristic(child, goal);

    %update cost of cell
    if cost_val < costs(child_x,child_y)
      costs(child_x,child_y) = cost_val;
      heuristics(child_x,child_y) = heuristic_val;
        
      %safe child's parent
      previous_x(child_x,child_y) = parent_x;
      previous_y(child_x,child_y) = parent_y;
    end
  end
end


parent = [goal(1), goal(2)];
xy = [];
while previous_x(parent(1), parent(2)) >= 0
  xy = [xy;[parent(1), parent(2)]];  
  child_y = previous_y(parent(1), parent(2));
  child_x = previous_x(parent(1), parent(2));
  child = [child_x, child_y];
  parent = child; 
end

%disp 'done'
%disp 'path cost: ', disp(costs(goal(1),goal(2)));
%disp 'path length: ', disp(distance2);
%disp 'number of nodes visited: ', disp(sum(closed_list(:)));

xy = [xy(:,1)*MAP.XWorldLimits(2)/MAP.GridSize(2),xy(:,2)*MAP.YWorldLimits(2)/MAP.GridSize(1)];
xy = flip(xy,1); 
