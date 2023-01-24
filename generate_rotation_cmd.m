function [w]=generate_rotation_cmd(angular_speed,robot_pose,goal_pos,robot_sample_rate)
    %Genera un vector de velocidades angulares proporcional al ángulo a recorrer
    robot_position=robot_pose(1:2);
    robot_orientation=robot_pose(3);
    displacement_vector=goal_pos(:)-robot_position(:);
    goal_angle=atan2(displacement_vector(2),displacement_vector(1));
    rotation_angle=angdiff(robot_orientation,goal_angle);
    cmd_length=int32(abs(rotation_angle)/(angular_speed*robot_sample_rate));
    w=repmat([sign(rotation_angle)*angular_speed],[cmd_length 1]);
end