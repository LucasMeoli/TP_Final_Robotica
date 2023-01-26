function particle_filter=create_particle_filter()
    const=Constants;
    particle_filter = robotics.ParticleFilter; %Creo objeto filtro de part�culas
    particle_filter.StateEstimationMethod = 'mean'; %Tomo el promedio de las part�culas como mi estado mas probable
    %Es muy ruidoso poner maxweight sobretodo con el theta, probablemente
    %esto tenga que ver con que el error cuadr�tico medio no distingue
    %angulos.
    %Se asigna la funci�n para actualizar la odometr�a
    particle_filter.StateTransitionFcn = @movement_model; 
    %Se asigna la funci�n del modelo de medici�n
    particle_filter.MeasurementLikelihoodFcn = @measurement_model; 
    particle_filter.ResamplingMethod = 'systematic'; % Remuestreo por SUS
    particle_filter.ResamplingPolicy.TriggerMethod = 'interval'; %Por cantidad de particulas efectivas
    particle_filter.ResamplingPolicy.SamplingInterval = const.particle_filter_resampling_interval
    
end