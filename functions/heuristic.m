function heur = heuristic(cell, goal)
    % M�nima ponderaci�n para la m�nima cantidad de nodos que tiene que visitar para encontrar un camino
    pond = 1; 
    heur = pond*norm(cell - goal);
end
