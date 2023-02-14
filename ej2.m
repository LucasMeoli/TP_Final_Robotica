%% Robot diferencial con lidar
% Robotica Movil - 2022 1c
% Funciona con MATLAB R2018a
close all
clear all
clc 

addpath utils
addpath functions

SIMULATE_LIDAR_NOISE = false; %simula datos no validos del lidar real, probar si se la banca
USE_ROOMBA = false;  % false para desarrollar usando el simulador, true para conectarse al robot real

%% Roomba
if USE_ROOMBA   % si se usa el robot real, se inicializa la conexion    
    init_connection()
end
    
%Objeto con constantes
const = Constants;
% creacion del Simulador de robot diferencial
diff_drive_obj = DifferentialDrive(const.wheel_separation,const.wheel_separation); 

%% Creacion del entorno
load maps/2022b_tp_map.mat
MAP_IMG = 1-double(imread('maps/2022b_tp_map.tiff'))/255;
map = robotics.OccupancyGrid(MAP_IMG, 25);

%% Parametros de la Simulacion
SIMULATION_DURATION = 3*60;          % Duracion total [s]
INIT_POS = random_empty_point(map);% [2,1,-pi/2]
%INIT_POS = [9, 9, -pi/2];

%% Crear sensor lidar en simulador
lidar = LidarSensor;
lidar.sensorOffset = const.lidar_offset;   
NUM_SCANS = 513/const.lidar_downsample_factor;
lidar.scanAngles = linspace(const.lidar_angle_start, const.lidar_angle_end, NUM_SCANS);
lidar.maxRange = const.lidar_max_range;

% Inicializar vectores de tiempo, entrada y pose
time_vec = 0:const.sample_time:SIMULATION_DURATION;     % Vector de Tiempo para duracion total
LOCATION_END = int32(2/const.sample_time);              %Iteraciones hasta ubicarse
pose = zeros(3,numel(time_vec));                        % Inicializar matriz de pose
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
slam_obj.MovementThreshold=[0.5,0.25];
state = "rotate";
est_map = [];
est_pose = [0,0,0];
orientation_angle = 0;
normal = [0,0];
path_blocked = true;
correct_orientation = false;
left = false;

% Bucle que itera sobre todo el tiempo de simulación
for time_step = 2:length(time_vec) 

    v_cmd = v_ref(time_step-1);   
    w_cmd = w_ref(time_step-1);
    
    %% a partir de aca el robot real o el simulador ejecutan v_cmd y w_cmd:
    if USE_ROOMBA       % para usar con el robot real
        
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
        ranges(ranges==0)=NaN; % lecturas erroneas y maxrange
        % Obtener pose del robot [x,y,yaw] de datos de odometrÃ­a (integrado por encoders).
        odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
        odomRotation = quat2eul(odomQuat);
        pose(:,time_step) = [odompose.Pose.Pose.Position.X + INIT_POS(1); odompose.Pose.Pose.Position.Y+ INIT_POS(2); odomRotation(1)];
    
    else        % para usar el simulador
   
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
    
    %Si alguna medicion da menor a 0.2 lo considero como NaN
    ranges(ranges<0.2) = NaN;
    %Obtengo la distancia minima de la medicion
    min_distance = distance_to_obstacle(ranges);
    if (correct_orientation == false)
        % Si un objeto se encuentra a mas de 50 cm sigo derecho
        if min_distance > const.obstacle_threshold 
            state = "move forward";
            path_blocked=false;
        elseif min_distance < const.obstacle_threshold
        % Si un objeto se encuentra a menos de 50 cm cambio de direccion
            state = "rotate";
            [orientation_angle] = calculate_orientation(ranges,path_blocked,left);
            path_blocked = true;
        end
    end
    
    correct_orientation = false;
    
    if (w_cmd == 0) && (mod(time_step,20) == 0)
        [wall_orientation] = calculate_orientation(ranges,false,left);
        orientation_error = abs(angdiff(wall_orientation,orientation_angle));
        
        if orientation_error>const.orientation_error_threshold 
            state="rotate";
            [orientation_angle] = calculate_orientation(ranges,path_blocked,left);
        end
        
    end
    
%     if time_step==900 && ~left 
%         state="rotate";
%         orientation_angle=-pi;
%         left = false
%         correct_orientation=false;
%     end
    
    if state=="move forward" 
        state;
        distance=min_distance;%distancia/sample_time=iteraciones
        speed_cmd=move_foward_command(distance);
        w_ref=w_ref(1:time_step-1);
        v_ref=v_ref(1:time_step-1);
        v_ref = [v_ref;speed_cmd(:,1)];
        w_ref = [w_ref;speed_cmd(:,2)];
        state="execute command";     
    elseif state=="rotate" && correct_orientation==false
       state;
       if path_blocked==false
          correct_orientation=true;
       end
       
       orientation_angle;
       speed_cmd=rotate_command(orientation_angle);
       w_ref=w_ref(1:time_step-1);
       v_ref=v_ref(1:time_step-1);
       v_ref = [v_ref;speed_cmd(:,1)];
       w_ref = [w_ref;speed_cmd(:,2)];
       state="execute command";
        
    elseif state=="execute command"
    end
    
    figure(1)
    if time_step==2 ||mod(time_step,10)==0
        
        ranges=process_measurement(slam_obj,ranges);
        [scans_slam,est_pose] = scansAndPoses(slam_obj);
        est_map = buildMap(scans_slam,est_pose,MAP_RES,const.lidar_max_range);
        
        show(est_map);
        hold on;
        scatter(est_pose(end,1),est_pose(end,2),'filled','LineWidth',2);
        plot(est_pose(:,1),est_pose(:,2),'LineWidth',2);
        quiver(est_pose(end,1),est_pose(end,2),cos(est_pose(end,3)),sin(est_pose(end,3)),0.5,'filled','LineWidth',2);
        axis([-23 23 -19 19])
        hold off;
        
    
    else
        visualizer(pose(:,time_step),ranges)
    end
        waitfor(robot_sample_rate);
end
