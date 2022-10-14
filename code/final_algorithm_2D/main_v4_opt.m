%% smoother optimization
path_up_opt = optimization(map,path_up);
path_up_opt = [path_up_opt;q_goal];
[path_smooth,keypoints] = kpsmoother(path_up_opt,map);
%% plot optimization figure
plot_figure_opt(2,map,q_init,q_goal,path,path_down,path_up,keypoints,path_smooth);
%plot_figure_opt(2,map,q_init,q_goal,path,path_down,path_up,[],[]);
disp_sum = cal_dist(path_smooth);
fprintf('Path Optimal Length 2st= %.3f \n\n',disp_sum);