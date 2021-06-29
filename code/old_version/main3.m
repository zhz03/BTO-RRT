
clear all
close all
%% map processing
I = imread('warran.png');
n = graythresh(I);
mm = im2bw(I,n);
MM = double(mm);
[a,b]=size(MM);
for i = 1:a
    for j= 1:b
        if MM(i,j) == 1
            MM(i,j) = 0;
        else MM(i,j) = 1;
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% main program
map = MM;
colormap=[1 1 1
          0 0 0
          1 0 0
          0 1 0
          0 0 1];
imshow(uint8(map),colormap)
hold on   % plot the map first
[map_h,map_w]=size(map); 
%task

q_init = [297,740]; % Y , X
q_goal = [ 966 300]; 
q_goal = [1013 430];
% q_init = [29,26]; % Y , X
% q_goal = [481 461];  
% q_init = [490,490]; % Y , X
% q_goal = [420 524];  
tic;  % tic-toc: Functions for Elapsed Time
%initial conditions
nodes = q_init;
Maxiterate = 10000000000;
stepsize = 30;
threshold = 20;
p = 0.5;
lines = [];q_rand = [];q_near = [];q_new = [];
%% main function
plot(q_init(2),q_init(1),'*b','LineWidth',4);
plot(q_goal(2),q_goal(1),'*r', 'LineWidth', 4);
for k = 1:Maxiterate
    arrived=is_goal_arrived(nodes,q_goal,threshold);
    if arrived
        nodes=[nodes;q_goal];
        lines = [lines;[size(nodes,1),size(nodes,1)-1]];
        break;
    end
    if rand <= p
        
        q_rand = [randi(map_h),randi(map_w)];
    else
        q_rand = q_goal;
        
        
    end
    if map( q_rand(1,1),q_rand(1,2) ) == 1 %map(1)height,map(2)width
        continue;
    end
    [q_new,q_near,q_near_ind,vector_dir] = get_qnew_qnear(stepsize,q_rand,nodes);
    add_qnew = is_add_in_veritces(map,q_new,q_near,vector_dir,15);
    if add_qnew
        nodes=[nodes;q_new];
        r_v = size(nodes,1);
        lines = [lines;[r_v,q_near_ind]];
    else
        continue;
    end
%     plot(q_near(1,1),q_near(2,1),'*b');
   plot([q_near(1,2),q_new(1,2)],[q_near(1,1),q_new(1,1)],'-b')
   drawnow
end
path =find_path_node(lines);
%plot base path
plot(nodes(path,2),nodes(path,1),'-r')
%smooth
path_smooth = smooth(path,nodes,map);
%plot smooth path
plot(nodes(path_smooth,2),nodes(path_smooth,1),'-g','LineWidth', 2)
%calculate the cumulative distance
disp = [nodes(path_smooth,2),nodes(path_smooth,1)];
[num,rol]=size(disp);
dissum = zeros(num-1,1);
for i=1:num-1
    dissum(i) = pdist2(disp(i,:),disp(i+1,:));
end
cum_dis = sum(dissum);
t2 =toc; 
fprintf('\n When delta_q = %d \n',stepsize);
fprintf('\nDistance Calculation = %f \n\n',cum_dis);
fprintf('Elapsed Time of Runing the program = %f\n\n',t2);


%% Test if goal is reached function
function arrived=is_goal_arrived(vertices,q_goal,threshold)

dist=pdist2(vertices(end,:),q_goal);
if dist <= threshold
    arrived=1;
else
    arrived=0;
end
end
%% get q_new and q_near function
function [q_new,q_near,q_near_ind,vector_dir] = get_qnew_qnear(delta_q,q_rand,nodes)

dist_rand = pdist2(nodes,q_rand);
[dist_min,q_near_ind]=min(dist_rand);
q_near=nodes(q_near_ind,:);
vector_dir =q_rand-q_near;
vector_dir = vector_dir./dist_min;
if dist_min > delta_q    
    q_new = floor( q_near+delta_q*vector_dir );
else
    q_new=q_rand;
end
end
%% decide if add q_new to the nodes of tree
function add_qnew = is_add_in_veritces(map,q_new,q_near,vector_dir,insert_p)

dist_new2near = norm(q_new - q_near);
dist_gap = dist_new2near/insert_p;
ii =1:insert_p;
insert_point = repmat(q_near,insert_p,1)+ii'.*dist_gap* vector_dir;
insert_point =[floor(insert_point);q_new];
insert_num = sub2ind(size(map),insert_point(:,1),insert_point(:,2));
or =find( map(insert_num)==1 );
if isempty(or)
    add_qnew=1;
else
    add_qnew=0;
end

end
%% find the node of path function
function path =find_path_node(edges)

e=edges(end,2);
path = edges(end,:);
while true
   ind= find(edges(:,1)==e);
    tmp_e = edges(ind,:);
    e=tmp_e(2);
    path=[path,e];
    if e==1
        break;
    end
end

end
%% use "Greedy approach" to smooth the path we find
function path_smooth = smooth(path,vertices,map)

path_smooth =path(end);
tmp_point = vertices(1,:);
while true
    l_p = length(path);
    for i=1:l_p
        vec = vertices( path(i),:) - tmp_point;
        vec_dir = vec/norm(vec);
        or_reduce = is_add_in_veritces(map ,vertices(path(i),: ),tmp_point,vec_dir,60);
        if or_reduce==1 
           path_smooth = [path_smooth, path(i)];
           tmp_point = vertices(path(i),: );
           break;
        else
            continue;
        end
    end
    vec_goal = vertices(end,:) - tmp_point;
    goal_dir = vec_goal/norm(vec_goal);
    or_goal = is_add_in_veritces(map , vertices(end,: ),tmp_point,goal_dir,60);
    if or_goal==1 
        path_smooth = [path_smooth, path(1)];
        break;
    else
        ind_path = find(path==path(i));
        path=path(1:ind_path);
    end
end

end

