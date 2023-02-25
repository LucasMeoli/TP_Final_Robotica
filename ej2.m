%% Ejercicio 2 - Partiendo de lo Desconocido
close all
clear all
clc 

addpath utils
addpath functions

SIMULATE_LIDAR_NOISE = false;       % Simula datos no validos del lidar real
USE_ROOMBA = false;                 % False para desarrollar usando el simulador, true para conectarse al robot real
DEBUG_PLOTS = true;
DEBUG_STATE = false;


%% Roomba

if USE_ROOMBA   % si se usa el robot real, se inicializa la conexion    
    init_connection();
end
    
%Objeto con constantes
const = Constants;
% creacion del Simulador de robot diferencial
diff_drive_obj = DifferentialDrive(const.wheel_separation,const.wheel_separation); 


%% Cargar mapa

load maps/2022b_tp_map.mat
MAP_IMG = 1-double(imread('maps/2022b_tp_map.tiff'))/255;

%load maps/obtained_map.mat
%MAP_IMG = 1-double(imread('maps/obtained_map.tiff'))/255;

map = robotics.OccupancyGrid(MAP_IMG,25);


%% Parametros de la Simulacion

% Duracion total [s]
SIMULATION_DURATION = 3 * 60;          
% Determinación de la posición inicial
INIT_POS = [9, 9, -pi/2];
%INIT_POS = [6, 4, pi/2];


%% Crear sensor lidar en simulador

lidar = LidarSensor;
lidar.sensorOffset = const.lidar_offset;   
NUM_SCANS = 513/const.lidar_downsample_factor;
lidar.scanAngles = linspace(const.lidar_angle_start, const.lidar_angle_end, NUM_SCANS);
lidar.maxRange = const.lidar_max_range;


%% Inicializar vectores de tiempo, entrada y pose

% Vector de Tiempo para duracion total
time_vec = 0:const.sample_time:SIMULATION_DURATION;
%Iteraciones hasta ubicarse
LOCATION_END = int32(2/const.sample_time);  
% Inicializar matriz de pose
pose = zeros(3,numel(time_vec));                       
pose(:,1) = INIT_POS;


%% Simulacion

robot_sample_rate = robotics.Rate(1/const.sample_time); %Para Matlab R2018b e inferiores

%Inicializo filtro de partícula
x_lims = map.XWorldLimits;
y_lims = map.YWorldLimits;
POSITION_LIMITS = [x_lims(2), x_lims(1); y_lims(2), x_lims(1); pi, -pi];

visualizer = Visualizer2D();
visualizer.mapName = 'map';
attachLidarSensor(visualizer, lidar);
release(visualizer);


%% Inicio del Objeto de matlab LidarSlam

%Genero comandos para localizarse
v_ref = zeros(1);
w_ref = zeros(1);

%Resolución del mapa [celdas/metro] 
MAP_RES = 25;             
% Defino la resolucon del mapa y el rango maximo del Lidar
slam_obj = robotics.LidarSLAM(MAP_RES, const.lidar_max_range);
% Defino las propiedas para el cierre de bucle del algoritmos. Es decir una
% vez que el robot vuelve a pasar por el mismo lugar.
% Umbral en la puntuación del algoritmo de coincidencia de exploración para
% aceptar cierres de bucle. Los umbrales más altos corresponden a una mejor
% coincidencia. 
slam_obj.LoopClosureThreshold = 400;
% Radio de búsqueda para detección de cierre de bucle. 
% Aumentar este radio afecta el rendimiento al aumentar el tiempo de búsqueda. 
slam_obj.LoopClosureSearchRadius = 2;
slam_obj.MovementThreshold = [0.5,0.25];


%% Inicialización del estado del robot y de las variables

state = "rotate";
estimated_map = [];
estimated_pose = [0,0,0];
angle_orientation = 0;
change_orientation = true;
correct_direction = false;
map_actualized = false;
angle_array = linspace(const.lidar_angle_start, const.lidar_angle_end, const.length_lidar_measurement);


%% Bucle que itera sobre todo el tiempo de simulación
for time_step = 2:length(time_vec) 

    v_cmd = v_ref(time_step-1);   
    w_cmd = w_ref(time_step-1);
    
    %% a partir de aca el robot real o el simulador ejecutan v_cmd y w_cmd:
    
    % Para usar con el robot real
    if USE_ROOMBA 
        
        % Enviar comando de velocidad
        cmdMsg.Linear.X = v_cmd;
        cmdMsg.Angular.Z = w_cmd;
        send(cmdPub,cmdMsg);
        
        % Recibir datos de lidar y odometrÃ­a
        scanMsg = receive(laserSub);
        odompose = odomSub.LatestMessage;
        
        % Obtener vector de distancias del lidar
        ranges_full = laserSub.LatestMessage.Ranges;
        ranges = ranges_full(1:const.lidar_downsample_factor:end);
        %ranges = circshift(ranges,length(ranges)/2);  % verificar
        ranges(ranges==0) = NaN; % lecturas erroneas y maxrange
        % Obtener pose del robot [x,y,yaw] de datos de odometrÃ­a (integrado por encoders).
        odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
        odomRotation = quat2eul(odomQuat);
        pose(:,time_step) = [odompose.Pose.Pose.Position.X + INIT_POS(1); odompose.Pose.Pose.Position.Y+ INIT_POS(2); odomRotation(1)]
    
    % Para usar el simulador
    else                                       
   
        % Mover el robot segun los comandos generados (ya vienen con ruido)
        [wL,wR] = inverseKinematics(diff_drive_obj,v_cmd,w_cmd);
        % Velocidad resultante
        [v,w] = forwardKinematics(diff_drive_obj,wL,wR);
        vel_robot = [v;0;w]; % velocidades en la terna del robot [vx;vy;w]
        vel_world = bodyToWorld(vel_robot,pose(:,time_step-1));  % Conversion de la terna del robot a la global
        % Realizar un paso de integracion
        pose(:,time_step) = pose(:,time_step-1) + vel_world*const.sample_time; 
        % Tomar nueva medicion del lidar
        ranges = lidar(pose(:,time_step));
        
        if SIMULATE_LIDAR_NOISE
            % Simular ruido de un lidar ruidoso (probar a ver si se la banca)
            chance_de_medicion_no_valida = 0.17;
            not_valid = rand(length(ranges),1);
            ranges(not_valid <= chance_de_medicion_no_valida) = NaN;
        end
    end
    
    % Si alguna medicion da menor a 0.2 lo considero como NaN
    ranges(ranges<0.2) = NaN;
    % Se obtiene la distancia minima de la medicion dentro de un rango
    % especifico de mediciones para evitar la colision del robot al avanzar
    min_distance = get_minimum_distance(ranges);
    
    if correct_direction == false
        % Si un objeto se encuentra a mas de 50 cm el robot avanza
        if min_distance > const.min_distance_to_obstacle 
            state = "advance";
            change_orientation = false;
        else
        % Si se encuentra a menos de 50 cm el robot cambia de dirección
            state = "rotate";
            [angle_orientation] = calculate_orientation(ranges,change_orientation);
            change_orientation = true;
        end
    else
        % Luego de cada procesamiento se inicializa como false
        correct_direction = false;
    end
    
    if (w_cmd == 0) && (mod(time_step,20) == 0)
        [wall_orientation] = calculate_orientation(ranges, false);
        orientation_error = abs(angdiff(wall_orientation, angle_orientation));
        
        if orientation_error > const.angle_orientation_error
            state = "rotate";
            [angle_orientation] = calculate_orientation(ranges, change_orientation);
        end
    end
    
    if DEBUG_STATE
        state
    end
     
    switch state
        case "advance" 
            w_ref = w_ref(1:time_step-1);
            v_ref = v_ref(1:time_step-1);
            
            speed_cmd = set_command(min_distance,false);
            v_ref = [v_ref;speed_cmd(:,1)];
            w_ref = [w_ref;speed_cmd(:,2)];
            state = "command";  
        case "rotate" 
           w_ref = w_ref(1:time_step-1);
           v_ref = v_ref(1:time_step-1);
           
           speed_cmd = set_command(angle_orientation,true);
           v_ref = [v_ref;speed_cmd(:,1)];
           w_ref = [w_ref;speed_cmd(:,2)];
           state = "command";

           if change_orientation == false
              correct_direction = true;
           end
    end
        
    % Definimos una taza de actualizacion para evitar actualizar el mapa en 
    % cada iteracion y de esta forma reducir el costo computacional
    if (mod(time_step,10) == 0)
        ranges = fillmissing(ranges, 'constant', 5); 
        scan = lidarScan(ranges, angle_array);
        addScan(slam_obj, scan);
        [scans_slam,estimated_pose] = scansAndPoses(slam_obj);
        estimated_map = buildMap(scans_slam,estimated_pose, MAP_RES, 25);
        map_actualized = true;
    else
        map_actualized = false;
    end
    
    if DEBUG_PLOTS
        figure(1)
        if map_actualized
            show(estimated_map);
            hold on;
            scatter(estimated_pose(end,1), estimated_pose(end,2), 'filled','LineWidth',2);
            plot(estimated_pose(:,1),estimated_pose(:,2),'LineWidth',2);
            quiver(estimated_pose(end,1),estimated_pose(end,2),cos(estimated_pose(end,3)),sin(estimated_pose(end,3)),...
                   0.5,'filled','LineWidth',2);
            axis([-23 23 -19 19])
            hold off;
        else
            visualizer(pose(:,time_step),ranges)
        end
    end
    
    waitfor(robot_sample_rate);
end

save('maps/obtained_map.mat','estimated_map','map');
mat = occupancyMatrix(estimated_map);
mat = 1-mat;

imwrite(mat(:, :, 1), 'maps/obtained_map.tiff');
for k = 2:size(mat, 3)
  imwrite(mat(:, :, k), 'maps/obtained_map.tiff', 'WriteMode', 'append');
end




