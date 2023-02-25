function heur = heuristic(cell, goal)
    % Mínima ponderación para la mínima cantidad de nodos que tiene que visitar para encontrar un camino
    pond = 1; 
    heur = pond*norm(cell - goal);
end
