function path_opt = optimization(map,path)
k = 0;
l_p = length(path(:,1)) ;
path_opt = [path(1,1),path(1,2)] ;
path_tem = [path(1,1),path(1,2)] ;
dis_tem = zeros(1,l_p) ;
for i = 2:l_p
    nodes = [path(i,1),path(i,2)];
    dis_tem(i) = pdist2(nodes,path_tem);
        if dis_tem(i)> 0 || dis_tem(i)> dis_tem(i-1)     
           indx = i;
           path_new = [path(indx,1) path(indx,2)];
        else path_new = [];
        end
    if (~isempty(path_new))
    if ~checkpath(path_tem,path_new,map)
        path_tem = [path(indx-1,1) path(indx-1,2)]; 
        path_opt = [path_opt; path_tem];
    end
    end
end

end