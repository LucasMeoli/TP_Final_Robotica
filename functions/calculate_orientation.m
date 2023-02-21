function [angle_orientation]=calculate_orientation(ranges, change_orientation)
% Funcion para determinar cuanto queremos que rote una vez que las
% mediciones determinaron que la distancia minima medida es menor al umbral
% establecido.

    const = Constants;
    ranges = fillmissing(ranges,'constant',5);

    % Determino el paso de las mediciones del LIDAR.
    step_meas = (const.lidar_angle_end - const.lidar_angle_start)/length(ranges);
    
    % Dependiendo si el robot se encuentra en la situacion de camino
    % bloqueado o no inicializo de distinta forma el indice de las
    % mediciones, el angulo de inicio y en angulo de orientacion.
    if change_orientation
        index_init_meas = int32(length(ranges)/2 - const.angle_range/(2*step_meas));
        angle_init_meas = -const.angle_range/2;
        angle_orientation = 0;
    else
        index_init_meas = 1;
        angle_init_meas = const.lidar_angle_start;
        angle_orientation = -pi/2; 
    end

    % Considero un angulo especifico de apertura para tomar como referencia
    % para despues girar en funcion del mismo.
    index_last_meas = index_init_meas + int32(const.angle_range/(step_meas));
    angle_last_meas = angle_init_meas + const.angle_range;      
    
    % Transforma las muestras de la primera y segunda medicion del angulo
    % especificado a cartesianas
    [x_init_meas, y_init_meas] = pol2cart(angle_init_meas, ranges(index_init_meas));
    [x_last_meas, y_last_meas] = pol2cart(angle_last_meas, ranges(index_last_meas));
    
    meas_init = [x_init_meas, y_init_meas];
    meas_last = [x_last_meas, y_last_meas];
    %Obtenemos el angulo que queremos que rote 
    angle_orientation = angle_orientation + atan2(meas_init(2) - meas_last(2), ...
                                                  meas_init(1) - meas_last(1));
end
