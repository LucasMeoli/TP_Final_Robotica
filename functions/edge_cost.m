function cost = edge_cost(parent, child, map)

  threshold = map.FreeThreshold; % Se podría elegir una probabilidad más alta pero se optó por una más conservativa
                   % para asegurarse de que no haya colisiones
  
  occ_prob = getOccupancy(map, [map.GridSize(1) - child(2), child(1)],'grid');
    
  if occ_prob >= threshold
      cost = inf;
  else
      cost = norm(parent-child) + 10*occ_prob;
  end
  %%% YOUR CODE FOR CALCULATING THE COST FROM VERTEX parent TO VERTEX child GOES HERE
  
end

