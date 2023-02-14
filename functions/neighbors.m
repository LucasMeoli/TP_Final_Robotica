function n = neighbors(cell, map_dimensions)

  n = [];
  pos_x = cell(1);
  pos_y = cell(2);
  size_x = map_dimensions(1);
  size_y = map_dimensions(2);
  
  for i = -1:1
      for j = -1:1
          x = pos_x + i;
          y = pos_y + j;
          
          if ((x < 1) || (x >= size_x))
              continue;
          elseif ((y < 1) || (y >= size_y))
              continue;
          elseif i==0 && j==0
              continue;
          end
          
          n = [n; [x,y] ]; %Se agregan los vecinos que cumplan con las condiciones de ser vecinos
      end
  end
  
  
  %%% YOUR CODE FOR CALCULATING THE NEIGHBORS OF A CELL GOES HERE
  
  % Return nx2 vector with the cell coordinates of the neighbors. 
  % Because planning_framework.m defines the cell positions as pos = [cell_y, cell_x],
  % make sure to return the neighbors as [n1_y, n1_x; n2_y, n2_x; ... ]
%%
  % Los comentarios de arriba informan que la posición en las celdas están definidas 
  % como pos = [cell_y, cell_x], pero si se implementa de esa forma no se llega al resultado esperado 
end

