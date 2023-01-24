function speed_cmd =move_foward_command(distance)
    const=Constants;
    cmd_length=int32(distance/(const.sample_time*const.linear_speed));
    v_cmd=const.linear_speed*ones(cmd_length,1);
    w_cmd=zeros(cmd_length,1);
    speed_cmd=[v_cmd,w_cmd];
end 