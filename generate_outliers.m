function new_particles=generate_outliers(particle_filter,pct_outliers,map)
%Genera puntos aleatorios en las particulas para no reducir la diversidad
%en el remuestreo
   total_outliers=int32(particle_filter.NumParticles*pct_outliers);
   outlier_index=randperm(particle_filter.NumParticles,total_outliers);
   new_particles=particle_filter.Particles;
   min_weight=min(particle_filter.Weights);
   for index =outlier_index
        new_particles(index,:)=random_empty_point(map);
        particle_filter.Weights(index)=min_weight;
   end
end