function heur = heuristic(cell, goal)
  pond = 1; % Mínima ponderación para la mínima cantidad de nodos que tiene que visitar para encontrar un camino
  heur = pond*norm(cell - goal);
  %%% YOUR CODE FOR CALCULATING THE REMAINING COST FROM A CELL TO THE GOAL GOES HERE
  
end

%%
% Para asegurarnos que la solución por A* es óptima, la función debe ser
% admisible, es decir, no debe sobreestimar el costo real hasta el
% objetivo.

%%
% Al usar h2 = pond * h se observa que al aumentar el valor de pond, se
% logra llegar al objetivo con menos cantidad de celdas recorridas. Se
% fuerza al algoritmo a encontrar el camino más cercano al objetivo. Se
% realiza menos exploración pero no se asegura que el resultado sea el
% camino más corto. 

