function mixed_cmd=generate_rotate_and_translation_cmd(linear_speed,angular_speed,robot_pose,goal_pos,robot_sample_rate)
    %Genera el comando mixto de rotacion y translación. Primero rota
    %después translada. 
    %Necesita la velocidad angular, la velocidad lineal, la pose estimada
    %del robot, su tasa de muestreo y la posición objetivo.
    v_cmd=generate_translation_cmd(linear_speed,robot_pose,goal_pos,robot_sample_rate);
    w_cmd=generate_rotation_cmd(angular_speed,robot_pose,goal_pos,robot_sample_rate);
    v_length_aux=length(v_cmd);
    v_cmd=[zeros(length(w_cmd),1);v_cmd];
    
    w_cmd=[w_cmd;zeros(v_length_aux,1)];
    mixed_cmd=[v_cmd,w_cmd];
end