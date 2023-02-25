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
          
          n = [n; [x,y] ]; 
      end
  end
end

