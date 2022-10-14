function distance = cal_dist(path)
len_path = length(path);
dis_path = zeros(len_path-1,1);
for i = 1: len_path-1
    dis_path(i) = pdist2(path(i,:),path(i+1,:));
end
distance = sum(dis_path);
end 