function plot_figure(fn,map,q_init,q_goal,path,path_down,path_up,keypoints,path_smooth)
figure(fn)
imshow(map); hold on;
%title("Trajectories after optimization");
plot(q_init(2),q_init(1),'*b','LineWidth',4);
plot(q_goal(2),q_goal(1),'*r', 'LineWidth', 4);
hold on 
if (~isempty(path))
% plot solution path
plot (path(:,2),path(:,1),'MarkerEdgeColor','k','LineWidth', 2);
%plot(path(:,2),path(:,1),'*k','LineWidth',4)
end

if (~isempty(path_down))
%plot downsample path
plot(path_down(:,2),path_down(:,1),'-b','LineWidth',2);
hold on
end

if (~isempty(path_up))
%plot upsample path  
plot(path_up(:,2),path_up(:,1),'-g','LineWidth',2);
hold on 
end

if (~isempty(keypoints))
% plot keypoints
plot(keypoints(:,1),keypoints(:,2), '*r','LineWidth',2);
end

if (~isempty(path_smooth))
% plot smooth path
plot(path_smooth(:,1),path_smooth(:,2), '-r','LineWidth',2);
end

end