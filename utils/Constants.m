 classdef Constants
    properties (Constant = true)
        %% Comunes
        
        % Constantes del robot
         wheel_radius = 0.072/3;             % Radio de las ruedas [mts]
         wheel_separation = 0.235;           % Distancia entre ruedas [mts] 
         lidar_offset = [0,0];               % Offset del LiDAR [mts]
         sample_time = 0.1;                  % Período de muestreo del robot [s]
         robot_diameter = 0.35;
         length_lidar_measurement = 171;
         
        % Constantes del LiDAR 
         lidar_downsample_factor = 3;        % Factor de submuestreo de los angulos del LiDAR
         lidar_angle_start = deg2rad(-90);   % Angulo donde inicia el barrido[rad]
         lidar_angle_end = deg2rad(90);      % Angulo donde finaliza el barrido[rad]
         lidar_max_range = 5;                % Rango máximo del LiDAR [mts]
        
        % Constantes de movimiento 
         angular_speed = 0.45;               % Velocidad angular [rad/s]
         linear_speed = 0.15;                % Velocidad linear [mts/s]
         delay_time = 3;
         path_update_interval = 5;           % Cantidad de pasos de correccion hasta actualizar A*
         
        % Constantes de tiempos
         location_end_time = 10;             % Tiempo para finalizar la localización [s]
         
        %% Ejercicio 1 - Regreso a casa 
         
        % Filtro de partículas
         particle_number = 300;                     % Cantidad de particulas [particulas]
         particle_filter_resampling_interval = 1;   % Pasos de correccion a realizar antes de remuestrear
         correction_interval = 15;                  % Cantidad de pasos de predicción (movimientos) hasta realizar una correccion
         outliers_pct = 0.03;                       % Porcentaje de partículas que se generan aletoriamente
         outilers_time_pct = 0.6                    % Porcentaje de tiempo en que se general particulas aleatoriamente
            
        %% Ejercicio 2 - Partiendo de lo desconocido
      
        % Constantes para eludir obstaculos
        min_distance_to_obstacle = 0.5;             % [m]
        angle_orientation_error = deg2rad(5);       % [Rad]
        angle_range = deg2rad(20);                  % [Rad]        
        
         
    end
 end