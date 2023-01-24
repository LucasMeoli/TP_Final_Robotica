%% Robot diferencial con lidar
% Robotica Movil - 2022 1c
% Funciona con MATLAB R2018a
close all
clear all
clc 

SIMULATE_LIDAR_NOISE = true; %simula datos no validos del lidar real, probar si se la banca
USE_ROOMBA = false;  % false para desarrollar usando el simulador, true para conectarse al robot real

%% Roomba
if USE_ROOMBA   % si se usa el robot real, se inicializa la conexion    
    init_connection()
end
    

%Objeto con constantes
const=Constants;
% creacion del Simulador de robot diferencial
diff_drive_obj = DifferentialDrive(const.wheel_separation,const.wheel_separation); 

%% Creacion del entorno
MAP_IMG = 1-double(imread('mapa_2022_1c.tiff'))/255;
map = robotics.OccupancyGrid(MAP_IMG, 25);

%% Crear sensor lidar en simulador
lidar = LidarSensor;
lidar.sensorOffset = const.lidar_offset;   
NUM_SCANS = 513/const.lidar_downsample_factor;
lidar.scanAngles = linspace(const.lidar_angle_start,const.lidar_angle_end,NUM_SCANS);
lidar.maxRange = const.lidar_max_range;

%% Crear visualizacion
visualizer = Visualizer2D();
visualizer.hasWaypoints=true;
visualizer.mapName = 'map';
attachLidarSensor(visualizer,lidar);
release(visualizer);

%% Parametros de la Simulacion
SIMULATION_DURATION = 3*60;          % Duracion total [s]
INIT_POS =random_empty_point(map);%[2.5; 1.5; -pi/2];         % Pose inicial (x y theta) del robot simulado (el robot pude arrancar en cualquier lugar valido del mapa)
GOAL_A = [1.5,1.3];
GOAL_B = [4.3,2.1];
WAYPOINTS=[GOAL_A;GOAL_B];

% Inicializar vectores de tiempo, entrada y pose
time_vec = 0:const.sample_time:SIMULATION_DURATION;     % Vector de Tiempo para duracion total
LOCATION_END = int32(20/const.sample_time);             %Iteraciones hasta ubicarse
pose = zeros(3,numel(time_vec));                        % Inicializar matriz de pose
pose(:,1) = INIT_POS;

%% Simulacion
robot_sample_rate = robotics.Rate(1/const.sample_time); %Para Matlab R2018b e inferiores

%Inicializo filtro de partículas
x_lims=map.XWorldLimits;
y_lims=map.YWorldLimits;
POSITION_LIMITS = [x_lims(2),x_lims(1);y_lims(2),x_lims(1);pi,-pi];
X_LIMS = x_lims;%[5,1];
Y_LIMS = y_lims;%[5,0];


particle_filter=create_particle_filter();
initialize(particle_filter,const.particle_number,POSITION_LIMITS)
particle_filter.Particles=initialize_particles(particle_filter,map);
path_to_B = [4.5,5.5]; %Inicializo en un punto que no puede estar nunca para que no falle cuando hago una comparación. Después este valor se tira

A_visited = false;
B_visited = false;
localized = false;
%%
%Genero comandos para localizarse
       % Velocidad angular a ser comandada
state="Locate";
v_ref = zeros(LOCATION_END,1);
w_ref = const.angular_speed*ones(LOCATION_END,1);
path=[];
path_counter=1;
v_cmd=0;
w_cmd=0;
correction_counter=1;
for time_step = 2:length(time_vec) % Itera sobre todo el tiempo de simulación
    
    
    if state=="Locate"              %Primer estado al que entra, las velocidades ya están cargadas en V_ref
        state
        state="Execute command";
        
    elseif state=="Plan path"       % Entra aca si pasaron n correcciones o si ya llego a A y tiene que calcular B
        state
        correction_counter=1;       % Avanza a Execute path despues de calcular la trayectoria
        state="Execute path";
        
        if A_visited==false
            path_to_A = A_star(MAP_IMG,robot_pos,GOAL_A);
            path = reduce_path(path_to_A);
        else
            display("Calcule B")
            path_to_B = A_star(MAP_IMG,robot_pos,GOAL_B);
            path = reduce_path(path_to_B);
        end
    elseif state=="Execute path"        %Ejecuta los caminos planeados en Plan path
        state
        if path_counter<=length(path)    %Si terminó de realizar el camino vuelve a plan path, si va a execute command 
            desired_location=path(path_counter,:);
            speed_cmd = generate_rotate_and_translation_cmd(const.angular_speed,const.angular_speed,robot_pos,desired_location,const.sample_time);
            v_ref = [v_ref;speed_cmd(:,1)];
            w_ref = [w_ref;speed_cmd(:,2)];
            path_counter=path_counter+1;
            state="Execute command";
        else
            state="Plan path";
            path_counter=1;
        end
    elseif state=="Execute command"     %Ejecuta los comandos de velocidad mientras haya comandos
                                        %Si llega a B sale va a exit, si realizó n correcciones va a plan path
                                        % y si se quedo sin comandos va a
                                        % execute path.
        if length(w_ref) >time_step
            v_cmd = v_ref(time_step);   
            w_cmd = w_ref(time_step);
            particle_filter.predict(v_cmd,w_cmd,const.sample_time);
                if mod(time_step,const.correction_interval) == 0
                    particle_filter.correct(ranges,map,const.lidar_max_range,"mse");
                    particle_filter.Particles = generate_outliers(particle_filter,const.outliers_pct,map);
                    correction_counter=correction_counter+1;
                end
            robot_pos=particle_filter.State;
            if norm(robot_pos(1:2)-GOAL_B)<0.2&&A_visited
                B_visited = true;
                state="Exit";
            elseif norm(robot_pos(1:2)-GOAL_A)<0.2 &&~A_visited
                A_visited=true;
                state="Delay";
            end
        elseif mod(correction_counter,5)==0 %Recalculo el path
            state="Plan path";
            display("Recalcule");
            %path_counter=1;
        else
            state="Execute path";
        end
    
    elseif state=="Delay"
        state
        speed_cmd = zeros(3/const.sample_time,2);
        v_ref = [v_ref;speed_cmd(1)];
        w_ref =[v_ref;speed_cmd(2)];
        state="Execute command";
    elseif state=="Exit"
        state
        break
    end
    if USE_ROOMBA       % para usar con el robot real
        
        % Enviar comando de velocidad
        %if state=="Execute command"
            cmdMsg.Linear.X = v_cmd;
            cmdMsg.Angular.Z = w_cmd;
        %else
            
        %    cmdMsg.Linear.X = 0;
        %    cmdMsg.Angular.Z = 0;
        %end
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
        if state=="Execute command"
            pose(:,time_step) = pose(:,time_step-1) + vel_world*const.sample_time;
        else
            
            pose(:,time_step) = pose(:,time_step-1);
        end
        %pose(:,time_step) = pose(:,time_step-1) + vel_world*const.sample_time; 
        % Tomar nueva medicion del lidar
        ranges = lidar(pose(:,time_step));
        if SIMULATE_LIDAR_NOISE
            % Simular ruido de un lidar ruidoso (probar a ver si se la banca)
            chance_de_medicion_no_valida = 0.17;
            not_valid = rand(length(ranges),1);
            ranges(not_valid <= chance_de_medicion_no_valida) = NaN;
        end
    end
    
    
    %Ejecuta los comandos de velocidad mientras haya comandos
    %Si llega a B sale va a exit, si realizó n correcciones va a plan path
    % y si se quedo sin comandos va a
                                    % execute path.
    
    
    
    %elseif length(w_ref) == time_step-1 %Tengo que mandar comandos 
    %    if B_visited == true
    %        disp 'Llegué al final';
    %        disp(time_step);
    %        break
    %    end
    %    plan_path = true;
        %goal = WAYPOINTS(1,:); %Objetivo en metros
    %    if correction_counter == 5
    %        if A_visited == false
    %            path_to_A = A_star(MAP_IMG,particle_filter.State(1:2),GOAL_A);
    %            path = reduce_path(path_to_A);
    %        else
    %            path_to_B = A_star(MAP_IMG,particle_filter.State(1:2),GOAL_B);
    %            path = reduce_path(path_to_B);
    %        end
    %        correction_counter=1;
    %        j=1;
    %    end
    %    goal = path(j,:);
    %    j = j+1;
    %    display("Planeo ruta")
    %    estimacion_estado = particle_filter.State;
        
    %    if plan_path == true  
    %        plan_path = false;          
    %        speed_cmd = generate_rotate_and_translation_cmd(const.angular_speed,const.angular_speed,estimacion_estado,goal,const.sample_time);
    %        if goal(1,1) == path_to_A(end,1) && goal(1,2) == path_to_A(end,2)
    %            A_visited = true;
    %            speed_cmd = [speed_cmd;zeros(3/const.sample_time,2)];
    %            disp 'Voy a esperar 3 segundos cuando llegue al A'
    %            path_to_B = A_star(MAP_IMG,particle_filter.State(1:2),GOAL_B); %Creo trayectoria desde el punto A al punto B
    %            path = reduce_path(path_to_B);
    %            j=1;
    %        end
    %        if goal(1,1) == path_to_B(end,1) && goal(1,2) == path_to_B(end,2)
    %            B_visited = true;
    %        end
    %        v_ref = [v_ref;speed_cmd(:,1)];
    %        w_ref = [w_ref;speed_cmd(:,2)];
    %    end
    %else
    %    particle_filter.predict(v_cmd,w_cmd,const.sample_time);
    %     if mod(time_step,const.correction_interval)==0
    %        particle_filter.correct(ranges,map,const.lidar_max_range,"gauss");
    %        correction_counter = correction_counter+1;
    %        particle_filter.Particles = generate_outliers(particle_filter,const.outliers_pct,map,X_LIMS,Y_LIMS);
    %     end
    %end

    markings=[WAYPOINTS;particle_filter.State(1:2)];
    visualizer(pose(:,time_step),markings,ranges)
    waitfor(robot_sample_rate);
    end
