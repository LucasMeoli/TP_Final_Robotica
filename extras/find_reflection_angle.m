function reflection_angle=find_reflection_angle(normal_vector,incident_vector)
    reflection_angle=atan2(normal_vector(2)-incident_vector(2),normal_vector(1)-incident_vector(1));
end