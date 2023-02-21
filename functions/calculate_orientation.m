function [orientation_angle]=calculate_orientation(ranges,blocked_path)
%Funciona para determinar cuanto queremos que rote una vez que las
%mediciones determinó que la distancia minima medida es menor al umbral
%establecido.

    const = Constants;
    ranges = fillmissing(ranges,'constant',5);

    % Determino el paso de las mediciones del LIDAR.
    lidar_angle_delta = (const.lidar_angle_end - const.lidar_angle_start)/length(ranges);
    
    % Dependiendo si el robot se encuentra en la situacion de camino
    % bloqueado o no inicializo de distinta forma el indice de las
    % mediciones, el angulo de inicio y en angulo de orientacion.
    if (blocked_path == true)
        first_sample_index = int32(length(ranges)/2 - const.samples_angle/(2*lidar_angle_delta));
        first_sample_angle = -const.samples_angle/2;
        orientation_angle = 0;
    else
        first_sample_index = 1;
        first_sample_angle = const.lidar_angle_start;
        orientation_angle = -pi/2; 
    end

    % Considero un angulo especifico de apertura para tomar como referencia
    % para despues girar en funcion del mismo.
    last_sample_index = first_sample_index + int32(const.samples_angle/(lidar_angle_delta));
    last_sample_angle = first_sample_angle + const.samples_angle;      
    
    % Transforma las muestras de la primera y segunda medicion del angulo
    % especificado a cartesianas
    [first_sample_x, first_sample_y] = pol2cart(first_sample_angle,ranges(first_sample_index));
    [last_sample_x, last_sample_y]   = pol2cart(last_sample_angle,ranges(last_sample_index));
    
    first_sample = [first_sample_x, first_sample_y];
    last_sample = [last_sample_x, last_sample_y];
    %Obtenemos el angulo que queremos que rote 
    orientation_angle = orientation_angle + atan2(first_sample(2) - last_sample(2), ...
                                                  first_sample(1) - last_sample(1));
end
