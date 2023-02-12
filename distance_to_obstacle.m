function min_distance = distance_to_obstacle(ranges)
    const = Constants;
    meas_index = (1:length(ranges))';
    % Trato NaN como lectura maxima
    ranges = fillmissing(ranges,'constant',5);
    % No deberian existir mediciones negativas
    ranges(ranges<0) = 0;
    mid_point = int32(length(ranges)/2);
    center_measurements = ranges(mid_point - const.obstacle_collision_deviation:mid_point ...
    + const.obstacle_collision_deviation);
    min_distance = min(center_measurements);
end