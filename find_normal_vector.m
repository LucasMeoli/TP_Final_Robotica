function normal_vector=find_normal_vector(point_a,point_b)
    
    diff_vector=[point_a(1)-point_b(1),point_a(2)-point_b(2)];
    normal_vector=[-diff_vector(2),diff_vector(1)];% (x,y).(-y,x)=-xy+xy=0
    normal_vector=normal_vector/norm(normal_vector);
end 