function xy = A_star(MAP_IMG,INIT_POS,GOAL)

    MAP = robotics.OccupancyGrid(MAP_IMG, 25);
    MAP.inflate(0.07);

    map_gridsize = MAP.GridSize;
    h = map_gridsize(1);
    w = map_gridsize(2);

    start = [round(INIT_POS(1)*MAP.GridSize(2)/MAP.XWorldLimits(2)), round(INIT_POS(2)*MAP.GridSize(1)/MAP.YWorldLimits(2))];
    parent = [start(1), start(2)];
    goal = [round(GOAL(1)*MAP.GridSize(2)/MAP.XWorldLimits(2)), round(GOAL(2)*MAP.GridSize(1)/MAP.YWorldLimits(2))];

    % Se inicializa la matriz de costos para cada celda con infinito
    costs = ones(w,h)*inf;
    costs(parent(1),parent(2)) = 0;
    
    % Se inicializa la matriz que contiene el costo estimado hasta llegar a
    %A y la matriz que contiene las celdas ya visitadas.
    heuristics = zeros(w,h);
    closed_list = zeros(w,h);

    % Se inicializan las matrices que contienen las posiciones del nodo
    % anterior.
    previous_x = zeros(w,h)-1;
    previous_y = zeros(w,h)-1;

    while (parent(1) ~= goal(1) || parent(2) ~= goal(2))
      % Se genera una mascara para asignar un infinito a las celdas
      % visitadas
      closed_mask = closed_list;
      closed_mask( closed_mask == 1 ) = Inf; 

      % Se busca el proximo candidato para la expansion
      open_list = costs + closed_mask + heuristics;

      % Se determina si existe una celda que no tenga cosot infinito
      if min(open_list(:)) == Inf
        disp 'no valid path found';
        break
      end

      % Se busca la celda de menor costo
      [x,y] = find(open_list == min(open_list(:)));
      parent_y = y(1);
      parent_x = x(1);
      parent = [parent_x, parent_y];

      closed_list(parent_x,parent_y) = 1;

      n = neighbors(parent, [w,h]);
      for i = 1:size(n,1)
        child_y = n(i,2);
        child_x = n(i,1);
        child = [child_x, child_y];
        % Se calcula el costo de llegar a cada celda
        cost_val = costs(parent_x,parent_y) + edge_cost(parent, child, MAP);
        % Se estima el costo desde la celda hasta la posicion final
        heuristic_val = heuristic(child, goal);
        % Se actualiza el costo de las celda
        if cost_val < costs(child_x,child_y)
          costs(child_x,child_y) = cost_val;
          heuristics(child_x,child_y) = heuristic_val;
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

    xy = [xy(:,1)*MAP.XWorldLimits(2)/MAP.GridSize(2),xy(:,2)*MAP.YWorldLimits(2)/MAP.GridSize(1)];
    xy = flip(xy,1); 
end
