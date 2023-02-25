function cost = edge_cost(parent, child, map)

  threshold = map.FreeThreshold; 
  
  occ_prob = getOccupancy(map, [map.GridSize(1) - child(2), child(1)],'grid');
    
  if occ_prob >= threshold
      cost = inf;
  else
      cost = norm(parent-child) + 10*occ_prob;
  end
  
end

