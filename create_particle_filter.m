function particle_filter=create_particle_filter()
    const = Constants;
    %Creo objeto filtro de partículas
    particle_filter = robotics.ParticleFilter; 
    %Tomo el promedio de las partículas como mi estado mas probable
    %Es muy ruidoso poner maxweight sobretodo con el theta, probablemente
    %esto tenga que ver con que el error cuadrático medio no distingue
    %angulos.
    particle_filter.StateEstimationMethod = 'mean'; 
    %Se asigna la función para actualizar la odometría
    particle_filter.StateTransitionFcn = @movement_model; 
    %Se asigna la función del modelo de medición
    particle_filter.MeasurementLikelihoodFcn = @measurement_model; 
    % Hay 4 remuestreo 'multinomial', 'systematic', 'stratified', 'residual'
    % No se que hace cada una 
    particle_filter.ResamplingMethod = 'systematic'; 
    %Por cantidad de particulas efectivas (puede ser 'ratio', 'interval')
    particle_filter.ResamplingPolicy.TriggerMethod = 'interval'; 
    particle_filter.ResamplingPolicy.SamplingInterval = const.particle_filter_resampling_interval
    
end