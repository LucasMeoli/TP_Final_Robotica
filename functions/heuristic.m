function heur = heuristic(cell, goal)
  pond = 1; % M�nima ponderaci�n para la m�nima cantidad de nodos que tiene que visitar para encontrar un camino
  heur = pond*norm(cell - goal);
  %%% YOUR CODE FOR CALCULATING THE REMAINING COST FROM A CELL TO THE GOAL GOES HERE
  
end

%%
% Para asegurarnos que la soluci�n por A* es �ptima, la funci�n debe ser
% admisible, es decir, no debe sobreestimar el costo real hasta el
% objetivo.

%%
% Al usar h2 = pond * h se observa que al aumentar el valor de pond, se
% logra llegar al objetivo con menos cantidad de celdas recorridas. Se
% fuerza al algoritmo a encontrar el camino m�s cercano al objetivo. Se
% realiza menos exploraci�n pero no se asegura que el resultado sea el
% camino m�s corto. 

