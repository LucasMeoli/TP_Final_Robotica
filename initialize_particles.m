function particles=initialize_particles(particle_filter,map)
    const=Constants;
    particles=zeros(const.particle_number,3);
    for i =1:const.particle_number
        particles(i,:)=random_empty_point(map);
    end
    
end