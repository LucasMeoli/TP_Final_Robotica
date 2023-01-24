function min_distance=distance_to_obstacle(ranges)
    meas_index=(1:length(ranges))';
    ranges=fillmissing(ranges,'linear'); %Trato nan interpolando
    ranges(ranges<0)=0;
    mid_point=int32(length(ranges)/2);
    const=Constants;
    center_measurements=ranges(mid_point-const.obstacle_collision_deviation:mid_point+const.obstacle_collision_deviation);
    min_distance=min(center_measurements);
end