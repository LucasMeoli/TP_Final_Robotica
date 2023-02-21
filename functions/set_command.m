function command = set_command(measurement, rotate)
    const = Constants;
    
    if rotate
        % Rotate Command
        rotation_angle = measurement;
        command_length = int32(abs(rotation_angle)/(const.sample_time*const.angular_speed));
        v_cmd = zeros(command_length,1);
        w_cmd = sign(wrapToPi(rotation_angle))*const.angular_speed*ones(command_length,1);
    else
        % Advance Command
        distance = measurement;
        command_length = int32(distance/(const.sample_time*const.linear_speed));
        v_cmd = const.linear_speed * ones(command_length,1);
        w_cmd = zeros(command_length,1);
    end
    
    command = [v_cmd,w_cmd];
end

