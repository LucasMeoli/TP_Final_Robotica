function reduced_path = reduce_path(xy)
n = 3;
reduced_path = xy(1 : n : end,:);
if reduced_path(end,:) ~= xy(end,:)
    reduced_path = [reduced_path;xy(end,:)];
end


