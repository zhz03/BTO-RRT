function [path_smooth,keypoints] = kpsmoother(path_up_opt,map) 
[rx,ry,x,y]=csp_cubic_spline(path_up_opt);

rgx = round(rx);
rgy = round(ry);

while check_collision(rgx,rgy,map)==1
    opt_path = smooth_optimizer(rgx,rgy,x,y,map);
    [rx,ry,x,y]=csp_cubic_spline(opt_path);
    rgx = round(rx);
    rgy = round(ry);  
end
keypoints = [x',y'];
path_smooth = [rx',ry'];
end
% figure(5)
% imshow(map); hold on;
% title("Path after Smoother optimization");
% plot(q_init(2),q_init(1),'*b','LineWidth',4);
% plot(q_goal(2),q_goal(1),'*r', 'LineWidth', 4);
% hold on 
% plot(path_up_opt(:,2),path_up_opt(:,1),'-g','LineWidth',2);
% hold on 
% plot(x(1,:),y(1,:),'*r','LineWidth',4);
% hold on 
% plot(rx(1,:),ry(1,:),'-b','LineWidth',2);


function flag = check_collision(rgx,rgy,map)
for i=1:length(rgx)
    tp = [rgy(i),rgx(i)];
    check = checkpoint(tp,map);
    if(check == 0)
       flag = 1; break;
    else flag =0;
    end
end

end

function  opt_path = smooth_optimizer(rgx,rgy,x,y,map)

search_points = [y',x'];
mid_points = create_mid(search_points);

Index = [];
count = 0;
for i=1:length(rgx)
   tp = [rgy(i),rgx(i)];
   check = checkpoint(tp,map);
   if(check == 0)
       indx = search_index_2d(mid_points,tp);
       if count == 0
          Index = [Index;indx];
          count = count +1;
       end 
       if Index(end) ~= indx
           Index = [Index;indx];
       end
   end
end

%search_points_new = insertion(search_points,mid_points,Index);

%function  tep = insertion(search_points,mid_points,Index)
midtep = search_points;
for i=1:length(Index)
    a = Index(i)+ i-1;
    b = Index(i);
    tep = [midtep(1:a,:)];
    tep = [tep;mid_points(b,:)];
    tep = [tep;midtep(a+1:end,:)];
    midtep = tep;
end
opt_path = midtep;
end

function mid_points = create_mid(points)
% points = n * 2 format [y,x]
dn = length(points);
mid_points = zeros(dn-1,2);
for i=1:(dn-1)
    mid_points(i,1) = (points(i,1) + points(i+1,1))/2;
    mid_points(i,2) = (points(i,2) + points(i+1,2))/2;
    %pdist2(points(i,:),points(i+1,:));
end
end

function ind = search_index_2d(search_points,target_point)
% search_points = n * 2 format [y,x]
dn = length(search_points);
dist = zeros(dn,1);
for i=1:dn
    dist(i) = pdist2(search_points(i,:),target_point);
    
end
 ind = find(dist==min(dist));
end
