function plot_figure(map,q_init,q_goal,path,path_down,path_up,path_smooth)
figure(2)
imshow(map); hold on;
title("Original Shortest path");

if (~isempty(path))
% plot solution path
plot (path(:,2),path(:,1),'MarkerEdgeColor','r','LineWidth', 3);
plot(path(:,2),path(:,1),'*r','LineWidth',6)
end

if (~isempty(path_down))
%plot downsample path
plot(path_down(:,2),path_down(:,1),'-b','LineWidth',3);
hold on
end

if (~isempty(path_up))
%plot upsample path  
plot(path_up(:,2),path_up(:,1),'-g','LineWidth',3);
hold on 
end

if (~isempty(path_smooth))
% plot smooth path
path_smooth = path_smooth';
plot(path_smooth(:,1),path_smooth(:,2), '-y','LineWidth',3);
end
plot(q_init(2),q_init(1),'*b','LineWidth',4);
plot(q_goal(2),q_goal(1),'*r', 'LineWidth', 4);
hold on 
end