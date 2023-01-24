function ranges=process_measurement(slam_obj,ranges)
    const=Constants;
    angles=linspace(const.lidar_angle_start,const.lidar_angle_end,length(ranges));
    meas_index=(1:length(ranges))';
    ranges=fillmissing(ranges,'linear'); %Trato nan interpolando
    ranges(ranges<0)=0;
    scan=lidarScan(ranges,angles);
    addScan(slam_obj,scan);
end