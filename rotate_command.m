function speed_cmd=rotate_command(rotation_angle)
    const=Constants;
    cmd_length=int32(abs(rotation_angle)/(const.sample_time*const.angular_speed));
    v_cmd=zeros(cmd_length,1);
    w_cmd=sign(wrapToPi(rotation_angle))*const.angular_speed*ones(cmd_length,1);
    speed_cmd=[v_cmd,w_cmd];
end 