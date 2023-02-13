function [v]=generate_translation_cmd(linear_speed,robot_pose,goal_pos,robot_sample_rate)
    %Genera un vector de velocidades proporcional a la distancia a recorrer
    robot_position=robot_pose(1:2);
    displacement_vector=robot_position-goal_pos;
    distance=norm(displacement_vector);
    cmd_length=int32(distance/(linear_speed*robot_sample_rate));
    v=repmat([linear_speed],[cmd_length 1]);
end