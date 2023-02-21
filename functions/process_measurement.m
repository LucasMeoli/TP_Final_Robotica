function ranges=process_measurement(slam_obj,ranges)
    const = Constants;
    angle_array = linspace(const.lidar_angle_start, const.lidar_angle_end, length(ranges));
    meas_index = (1:length(ranges))';
    ranges = fillmissing(ranges, 'constant', 5); 
    scan = lidarScan(ranges, angle_array);
    addScan(slam_obj, scan);
end