function particle_filter=create_particle_filter()
    const=Constants;
    particle_filter = robotics.ParticleFilter; %Creo objeto filtro de partículas
    particle_filter.StateEstimationMethod = 'mean'; %Tomo el promedio de las partículas como mi estado mas probable
    %Es muy ruidoso poner maxweight sobretodo con el theta, probablemente
    %esto tenga que ver con que el error cuadrático medio no distingue
    %angulos.
    particle_filter.StateTransitionFcn = @movement_model; %Función para actualizar la odometría
    particle_filter.MeasurementLikelihoodFcn = @measurement_model; %Función del modelo de medición
    particle_filter.ResamplingMethod = 'systematic'; % Remuestreo por SUS
    particle_filter.ResamplingPolicy.TriggerMethod = 'interval'; %Por cantidad de particulas efectivas
    particle_filter.ResamplingPolicy.SamplingInterval = const.particle_filter_resampling_interval
    
end