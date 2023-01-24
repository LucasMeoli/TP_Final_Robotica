%Función para calcular el likelikhood de la medicion de cada partícula
%es llamada por robotics.ParticleFilter.correct()
%Permite calcular el likelhood con el modelo de medición o con el error
%cuadrático medio
%¿Como tratar a los NaN?
function likelihood = measurement_model(particle_filter,predicted_particles,measurement,varargin)
    SIGMA_MODEL = 0.1;%0.5;
    ANGLE_LENGTH = 15; %Me quedo con sólo estos angulos
    DOWNSample_FACTOR = ceil(length(measurement)/ANGLE_LENGTH);
    map = varargin{1};
    max_range = varargin{2};
    
    method = varargin{3};
    %display("Calcule el Likelihood de la medición")
    
    angles = linspace(-pi/2,pi/2,ANGLE_LENGTH); %Esto esta hardcodeado
    
    measurement = downsample(measurement,DOWNSample_FACTOR); %Sub muestreo las mediciones
    likelihood = ones(particle_filter.NumParticles,1);
    meas_index=(1:length(measurement))';
    measurement=fillmissing(measurement,'linear'); %Trato nan interpolando
    x_lims = map.XWorldLimits;
    y_lims = map.YWorldLimits;
    for row = 1:particle_filter.NumParticles
        particle_position = predicted_particles(row,:);
        if particle_position(1) <= x_lims(1) || particle_position(1) >= x_lims(2)...
                ||  particle_position(2) <= y_lims(1) || particle_position(2) >= y_lims(2)   
            likelihood(row)=0;
        elseif getOccupancy(map,particle_position(1:2)) < map.FreeThreshold
            ray_intercept_point = rayIntersection(map,particle_position,angles,max_range);
            particle_measurement = sqrt(sum((ray_intercept_point-particle_position(1:2)).^2,2));
            particle_measurement(isnan(particle_measurement)) = max_range;
            if method == "mse"
                likelihood(row) = 1/immse(measurement,particle_measurement);
            elseif method == "gauss"
                for index = 1:ANGLE_LENGTH
                    if ~isnan(measurement(index)) && ~isnan(particle_measurement(index)) 
                        likelihood(row)=likelihood(row)*(normpdf(measurement(index),particle_measurement(index),SIGMA_MODEL));
                    elseif ~(isnan(measurement(index)) && isnan(measurement(index))) 
                        %Si uno es NaN y el otro no bajo el likelihood
                       likelihood(row) = likelihood(row)*0.00000001; %Bajo likelihood
                    else
                        likelihood(row) = likelihood(row)*0.0000001;

                    end
                end
            end
        else
            likelihood(row) = 0;
        end
        likelihood = likelihood;%Normalizo ->?
    end
end