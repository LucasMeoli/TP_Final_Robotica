function min_distance = get_minimum_distance(ranges)
% Funcion para obtener la distancia minima de las mediciones de LIDAR. Se
% considera unicamente un angulo de apertura en frente del robot. Es decir
% a partir del centro del robot se toman las mediciones 15 grados para cada
% lado. De esta forma solo se consideran los obstaculos que se encuentran
% delante del robot y no al costado.

    const = Constants;
    meas_index = (1:length(ranges))';
    % Trato NaN como lectura maxima
    ranges = fillmissing(ranges,'constant',5);
    % No deberian existir mediciones negativas
    ranges(ranges<0) = 0;
    
    % Rango de mediciones del lidar para evitar colisiones. Se debe
    % tener en cuenta que aproximadamente cada step del LIDAR es
    % aproximadamente 1.05 grados, por lo cual equivalen a 20
    % grados aproximadamente para cada lado.
    collision_deviation = ceil(atand(const.robot_diameter/(2*const.min_distance_to_obstacle)));
    % Se tiene en cuenta la distancia minima y el diametro del robot para
    % obtener el angulo determinado para evitar colisiones
    
    
    % Obtengo el punto central de las mediciones de LIDAR
    center_point = int32(length(ranges)/2);
    % Me centro en el las mediciones centrales del robot
    center_measurements = ranges(center_point - collision_deviation:center_point + collision_deviation);

    min_distance = min(center_measurements);
end