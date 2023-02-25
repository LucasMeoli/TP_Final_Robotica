function likelihood = measurement_model(particle_filter, predicted_particles, measurement,varargin)

    SIGMA_MODEL = 0.1; 
    ANGLE_LENGTH = 15; 
    DOWNSample_FACTOR = ceil(length(measurement)/ANGLE_LENGTH);
    
    % Obtengo los datos del mapa (PO = 0.65 y PD = 0.2 ) 
    map = varargin{1};
    % Obtengo el valor maximo de medicion que es 5 
    max_range = varargin{2};
  
    % Disminuyo la cantidad de mediciones para reducir el tiempo
    measurement = downsample(measurement,DOWNSample_FACTOR);
    
    % Creo el arreglo deangulos desde -pi a pi con igual cantidad de
    % muestras las mediciones resampleadas
    angles = linspace(-pi/2 ,pi/2 , length(measurement)); 
    % inicializo el vector del likelihood
    likelihood = ones(particle_filter.NumParticles,1);
    % Creo el vector de index (vector del 1 al 171)
    meas_index = (1:length(measurement))';
    
    measurement = fillmissing(measurement,'constant',5); 

    x_lims = map.XWorldLimits;
    y_lims = map.YWorldLimits;
    
    % Se hace un bucle donde se pasa por cada una de las particulas del filtro
    for row = 1:particle_filter.NumParticles
        particle_position = predicted_particles(row,:);
        
        if particle_position(1) <= x_lims(1) || particle_position(1) >= x_lims(2) ||...
           particle_position(2) <= y_lims(1) || particle_position(2) >= y_lims(2) 
            % Si encuentran fuera de los limites entonces su likelihood = 0
            likelihood(row)=0;
            
        elseif getOccupancy(map, particle_position(1:2)) < map.FreeThreshold
            % Si la particula se encuentra en una posicion libre ahi calculo
            % su medicion, sino no ya que el robot no puede estar ahi
            % De cada particula simula rayos y devuelve la interseccion en
            % coordenadas de la intereseccion con celdas ocupadas
            ray_intercept_point = rayIntersection(map, particle_position, angles, max_range);
            % Obtengo la distancia de la medicion a partir de las coordenadas
            particle_measurement = sqrt(sum((ray_intercept_point-particle_position(1:2)).^2,2));
            % Transformo los Nan en el valor maximo de medicion
            particle_measurement(isnan(particle_measurement)) = max_range;
            %particle_measurement = downsample(particle_measurement,DOWNSample_FACTOR);
            
            likelihood(row) = 1/immse(measurement,particle_measurement);
        else
            likelihood(row) = 0;
        end 
        
        likelihood = likelihood;
    end 
end