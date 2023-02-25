%% Robot diferencial con lidar
% Robotica Movil - 2022 2c
close all
clear all
clc 

addpath utils
addpath functions

SIMULATE_LIDAR_NOISE = true;                    % Simula datos no validos del lidar real, probar si se la banca
USE_ROOMBA = false;                             % False para desarrollar usando el simulador, true para conectarse al robot real
DEBUG = false;                                  % Imprime logs utiles para debugging

%% Roomba

if USE_ROOMBA                                   % Si se usa el robot real, se inicializa la conexion    
    init_connection();
end
    

%% Definiciones del robot

const = Constants;
% creacion del Simulador de robot diferencial
diff_drive_obj = DifferentialDrive(const.wheel_separation, const.wheel_separation); 


%% Creacion del entorno

% Mapa 2022 Primer Cuatrimestre
% MAP_IMG = 1-double(imread('maps/imagen_2021_2c_mapa_tp.tiff'))/255;

% Mapa 2022 Segundo cuatrimestre
% load 2022b_tp_map.mat
MAP_IMG = 1-double(imread('maps/2022b_tp_map.tiff'))/255;

map = robotics.OccupancyGrid(MAP_IMG, 25);

% Limites del mapa
x_lims = map.XWorldLimits;
y_lims = map.YWorldLimits;
POSITION_LIMITS = [x_lims(2), x_lims(1); 
                   y_lims(2), x_lims(1); 
                   pi, -pi];


%% Crear sensor lidar en simulador

lidar = LidarSensor;
lidar.sensorOffset = const.lidar_offset;   
NUM_SCANS = 513/const.lidar_downsample_factor;
lidar.scanAngles = linspace(const.lidar_angle_start, const.lidar_angle_end, NUM_SCANS);
lidar.maxRange = const.lidar_max_range;


%% Crear visualizacion

visualizer = Visualizer2D();
visualizer.hasWaypoints = true;
visualizer.hasParticles = true;
visualizer.mapName = 'map';
attachLidarSensor(visualizer, lidar);
release(visualizer);


%% Parametros de la simulacion

SIMULATION_DURATION = 3*60;                     % Duracion total [s]
INIT_POS = random_empty_point(map);             % Pose inicial (x y theta) del robot simulado (el robot pude arrancar en cualquier lugar valido del mapa)
INIT_POS = [9; 9; -pi/2];                       % Pose inicial dada por el profesor
%INIT_POS = [9; 15; -pi/2];           

%GOAL_A = [12, 8];
GOAL_A = [5.3, 4.3];
WAYPOINTS = [GOAL_A];


%% Inicializar vectores de tiempo, entrada y pose

% Vector de Tiempo para duracion total
time_vec = 0:const.sample_time:SIMULATION_DURATION;    
% Iteraciones hasta ubicarse 
% 60 se podria considerar como 3 vueltas sobre su ejere aproximadamente
LOCATION_END = int32(40/const.sample_time);             
pose = zeros(3, numel(time_vec));                % Inicializar matriz de pose
pose(:, 1) = INIT_POS;


%% Simulacion

robot_sample_rate = robotics.Rate(1/const.sample_time);     % Para Matlab R2018b e inferiores

% Inicializo filtro de particulas
particle_filter = create_particle_filter();
initialize(particle_filter, const.particle_number, POSITION_LIMITS)
particle_filter.Particles = initialize_particles(particle_filter, map);

% Inicializo variables
state = "Locate";                               % Posibles estados: "Locate", "Plan path", "Execute path", "Execute command" y "Exit"
v_ref = zeros(LOCATION_END, 1);
w_ref = const.angular_speed*ones(LOCATION_END, 1);
path = [];
path_counter = 1;
v_cmd = 0;
w_cmd = 0;
correction_counter = 1;

% Maquina de estados
for time_step = 2:length(time_vec)              % Itera sobre todo el tiempo de simulacion
    
    switch state

        % Primer estado al que entra, las velocidades ya estan cargadas en V_ref
        case "Locate"
            if DEBUG
                state
            end

            state = "Execute command";
        
        % Ejecuta los comandos de velocidad mientras haya comandos
        % Si llega a B sale va a exit, si realizo n correcciones va a plan path y si se quedo sin comandos va a execute path.
        case "Execute command"
            if DEBUG
                state
            end
            
            if length(w_ref) > time_step
                v_cmd = v_ref(time_step);   
                w_cmd = w_ref(time_step);
                particle_filter.predict(v_cmd, w_cmd, const.sample_time);
                    if mod(time_step, const.correction_interval) == 0
                        particle_filter.correct(ranges, map, const.lidar_max_range, "mse");
                        if time_step > int32(LOCATION_END * const.outilers_time_pct) 
                            outliers_pct = 0;
                        else
                            outliers_pct = 0.1;
                        end
                        particle_filter.Particles = generate_outliers(particle_filter, outliers_pct, map);
                        correction_counter = correction_counter+1;
                    end
                robot_pos = particle_filter.State;
                if norm(robot_pos(1:2)-GOAL_A) < 0.2
                    state = "Exit";
                end
            elseif mod(correction_counter,5) == 0       % Recalculo el path
                state = "Plan path";
                if DEBUG
                    display("Recalcule");
                end
            else
                state = "Execute path";
            end
        
        % Entra aca si pasaron n correcciones o si ya llego a A y tiene que calcular B
        case "Plan path"
            if DEBUG
                state
            end

            correction_counter = 1;                     % Avanza a Execute path despues de calcular la trayectoria
            state = "Execute path";
            
            path_to_A = A_star(MAP_IMG, robot_pos, GOAL_A);
            path = reduce_path(path_to_A);
        
        % Ejecuta los caminos planeados en Plan path
        case "Execute path"
            if DEBUG
                state
            end

            if path_counter <= length(path)               % Si termino de realizar el camino vuelve a plan path, si va a execute command 
                desired_location = path(path_counter, :);
                speed_cmd = generate_rotate_and_translation_cmd(const.angular_speed, const.angular_speed, robot_pos, desired_location, const.sample_time);
                v_ref = [v_ref; speed_cmd(:, 1)];
                w_ref = [w_ref; speed_cmd(:, 2)];
                path_counter = path_counter + 1;
                state = "Execute command";
            else
                state = "Plan path";
                path_counter = 1;
            end
               
        case "Exit"
            if DEBUG
                state
            end

            break;

        otherwise
            break;
    end
    
    % Para usar con el robot real
    if USE_ROOMBA                                   
        
        % Enviar comando de velocidad
        cmdMsg.Linear.X = v_cmd;
        cmdMsg.Angular.Z = w_cmd;
        send(cmdPub,cmdMsg);
        
        % Recibir datos de lidar y odometria
        scanMsg = receive(laserSub);
        odompose = odomSub.LatestMessage;
        
        % Obtener vector de distancias del lidar
        ranges_full = laserSub.LatestMessage.Ranges;
        ranges = ranges_full(1:const.lidar_downsample_factor:end);
        ranges(ranges==0)=NaN;                                      % Lecturas erroneas y maxrange
        % Obtener pose del robot [x,y,yaw] de datos de odometria (integrado por encoders).
        odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
        odomRotation = quat2eul(odomQuat);
        pose(:,time_step) = [odompose.Pose.Pose.Position.X + INIT_POS(1); odompose.Pose.Pose.Position.Y+ INIT_POS(2); odomRotation(1)];
    
    % Para usar el simulador
    else                                       
   
        % Mover el robot segun los comandos generados (ya vienen con ruido)
        [wL,wR] = inverseKinematics(diff_drive_obj,v_cmd,w_cmd);
        % Velocidad resultante
        [v,w] = forwardKinematics(diff_drive_obj,wL,wR);
        vel_robot = [v;0;w];                                        % Velocidades en la terna del robot [vx;vy;w]
        vel_world = bodyToWorld(vel_robot,pose(:,time_step-1));     % Conversion de la terna del robot a la global
        % Realizar un paso de integracion
        if state == "Execute command"
            pose(:,time_step) = pose(:,time_step-1) + vel_world*const.sample_time;
        else
            pose(:,time_step) = pose(:,time_step-1);
        end
        % Tomar nueva medicion del lidar
        ranges = lidar(pose(:,time_step));
        if SIMULATE_LIDAR_NOISE
            % Simular ruido de un lidar ruidoso (probar a ver si se la banca)
            chance_de_medicion_no_valida = 0.17;
            not_valid = rand(length(ranges),1);
            ranges(not_valid <= chance_de_medicion_no_valida) = NaN;
        end
    end
    
    show_particles = [particle_filter.Particles(:,1), particle_filter.Particles(:,2)];
    %markings = [WAYPOINTS; particle_filter.State(1:2); show_particles];
    markings = [WAYPOINTS; particle_filter.State(1:2)];
    visualizer(pose(:,time_step), show_particles, markings, ranges)
    waitfor(robot_sample_rate);

end
