clear all;
%% load settings
name = 'map8';
type = '.png';
fullname = [name,type];
map=im2bw(imread(fullname)); 


% map1-3
q_init=[10 10]; % source position in Y, X format
q_goal=[490 490]; % goal position in Y, X format

% map4-6
q_init=[245 10]; % source position in Y, X format
q_goal=[245 490]; % goal position in Y, X format
% map4
q_init=[50 200 ]; % source position in Y, X format
q_goal=[520 400 ]; % goal position in Y, X format
% map7-8
q_init=[10 490]; % source position in Y, X format
q_goal=[490 10]; % goal position in Y, X format
%% algorithm settings
stepsize=20; % size of each step of the RRT
disTh=10; % nodes closer than this threshold are taken as almost the same
maxFailedAttempts = 10000;

%% algorithm main
figure(1)
imshow(map); hold on;
title("Path generated by BTO-RRT algorithm core");
plot(q_init(2),q_init(1),'*b','LineWidth',4);
plot(q_goal(2),q_goal(1),'*r', 'LineWidth', 4);
hold on
N = 10;
Dist_RRT = zeros(1,N);
for i=1:N
[time,path,path_down,path_up]= main_comb(map,q_init,q_goal,stepsize,disTh,maxFailedAttempts);
dist_RRT = cal_dist(path);
dist_down = cal_dist(path_down);
Dist_RRT(i) = dist_RRT;
Dist_down(i) = dist_down;
end

% dist_up = cal_dist(path_up);
% fprintf('Path original Length = %.3f \n\n',dist_RRT);
% fprintf('Path Optimal Length downsample= %.3f \n\n',dist_down);
% fprintf('Path Optimal Length upsample= %.3f \n\n',dist_up);

foldername = 'downsample\';
RRTname = '_RRT';
DSname = '_DS_down';
mattype = '.mat';
fullname1 = [[[foldername,name],RRTname],mattype];
fullname2 = [[[foldername,name],DSname],mattype];
save(fullname1,'Dist_RRT'); 
save(fullname2,'Dist_down');  

% %% plot figure
% if (~isempty(path))
% % plot solution path
% plot (path(:,2),path(:,1),'MarkerEdgeColor','r','LineWidth', 2);
% plot(path(:,2),path(:,1),'*r','LineWidth',4)
% end
% 
% 
% %% smoother optimization
% tic;
% path_up_opt = optimization(map,path_up);
% path_up_opt = [path_up_opt;q_goal];
% [path_smooth,keypoints] = kpsmoother(path_up_opt,map);
% time1 = toc;
% %% plot optimization figure
% plot_figure_opt(2,map,q_init,q_goal,path,path_down,path_up,keypoints,path_smooth);
% 
% disp_sum = cal_dist(path_smooth);
% fprintf('\nTime elapse=%f \n', time); 
% fprintf('\nTime elapse=%f \n', time1);
% fprintf('Path Optimal Length 2st= %.3f \n\n',disp_sum);


% % %% smoother
% [rx,ry,x,y]=csp_cubic_spline(path_up_opt);
% figure(4)
% imshow(map); hold on;
% plot(q_init(2),q_init(1),'*b','LineWidth',4);
% plot(q_goal(2),q_goal(1),'*r', 'LineWidth', 4);
% hold on 
% plot(path_up_opt(:,2),path_up_opt(:,1),'-g','LineWidth',2);
% hold on 
% plot(x(1,:),y(1,:),'*r','LineWidth',4);
% hold on 
% plot(rx(1,:),ry(1,:),'b','LineWidth',1);
% title("Intermediate result");
% %% calculate distance
% pathOrigin=0;
% for i=1:length(path)-1, pathOrigin=pathOrigin+pdist2(path(i,1:2),path(i+1,1:2)); end
% dis_sum = cal_dist(path_down);
% disp_sum = cal_dist(path_up);


% %% display distance 
% % Display info we need
% fprintf('\nTime elapse=%f \n', time); 
% fprintf('Path Original Length=%.3f \n',pathOrigin);
% fprintf('Path Optimal Length 1st= %.3f \n',dis_sum);
% fprintf('Path Optimal Length 2st= %.3f \n\n',disp_sum);