clc
clear all
close all
%% map processing
I = imread('map1.png');
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

[map_h,map_w] = size(map); 
%task
q_start = [54,24]; %x,y
q_goal = [ 483 448]; %x,y  
tic;  % tic-toc: Functions for Elapsed Time
%initial conditions
nodes = q_start;
Maxiterate = 10000;  
stepsize = 50; 
threshold = 20;
p = 0.5;
lines = [];q_rand = [];q_near = [];q_new = [];
%plot task points
plot(q_start(1),q_start(2),'*b','LineWidth',4)
plot(q_goal(1),q_goal(2),'*r', 'LineWidth', 4)

%main loop
for k = 1:Maxiterate
    reached = goal_reach(nodes,q_goal,threshold);
    if reached 
        nodes = [nodes;q_goal];
        lines = [lines;[size(nodes,1),size(nodes,1)-1]];
    end
    if rand <= p
        q_rand = q_goal;
    else
        q_rand = [randi(map_h),randi(map_w)];
        
    end
    if map( q_rand(1,1),q_rand(1,2) ) == 1
        continue;
    end
    [q_new,q_near,q_nearindx,dir] = get_qnew_qnear(q_rand,nodes,stepsize);
    interp = 10;
    qnew_add = add_into_nodes(interp,map,q_new,q_near,dir);
    if qnew_add
        nodes = [nodes;q_new];
        n_s = size(nodes,1);
        lines = [lines; [n_s , q_nearindx]];
    else
        continue;
    end
    plot([q_near(1,1),q_new(1,1)],[q_near(1,2),q_new(1,2)],'-b')
    drawnow
    
end
path =find_path_node(lines);
%smooth
path_smooth = smooth(path,nodes,map);
%plot smooth path
plot(nodes(path_smooth,2),nodes(path_smooth,1),'g','LineWidth', 2);
%calculate the cumulative distance
disp = [nodes(path_smooth,2),nodes(path_smooth,1)];
[num,rol]=size(disp);
dissum = zeros(num-1,1);

for i=1:num-1
    dissum(i) = pdist2(disp(i,:),disp(i+1,:));
end
cum_dis = sum(dissum);
t2 =toc; 
fprintf('\n When delta_q = %d \n',delta_q);
fprintf('\nDistance Calculation = %f \n\n',cum_dis);
fprintf('Elapsed Time of Runing the program = %f\n\n',t2);

%% Test if goal is reached function
function reached =  goal_reach(nodes,q_goal,threshold)
dist = distance(nodes,q_goal);
if dist <= threshold
    reached=1;
else
    reached=0;
end

end

%% distance
function h=distance(a,b)
h = sqrt((a(end,1)-b(:,1)).^2 + (a(end,2)-b(:,2)).^2 );
end 
%% get q_new and q_near function
function [q_new,q_near,q_nearindx,dir] = get_qnew_qnear(q_rand,nodes,stepsize)

dist_rand = distance(nodes,q_rand);
[dist_min,q_nearindx] = min(dist_rand);
q_near = nodes(q_nearindx,:);
dir = q_rand-q_near;
dir = dir./dist_min;
if dist_min > stepsize   
    q_new = floor( q_near+stepsize*dir );
else
    q_new = q_rand;
end

end

%% decide if add q_new to the nodes of tree
function qnew_add = add_into_nodes(interp,map,q_new,q_near,dir)

dist_new2near = norm(q_new - q_near);
dist_set = dist_new2near / interp;
num = 1:interp;
insert_point = repmat(q_near,interp,1)+num'.* dist_set * dir;
insert_point =[floor(insert_point);q_new];
insert_num = sub2ind(size(map),insert_point(:,1),insert_point(:,2));
log = find(map(insert_num) == 1);
if isempty(log)
    qnew_add=1;
else
    qnew_add=0;
end

end

%% find the node of path function
function path =find_path_node(lines)

en=lines(end,2);
path =lines(end,:);
while true
   indx= find(lines(:,1)==en);
   tmp_e = lines(indx,:);
   en=tmp_e(2);
   path=[path,en];
   if en==1
       break;
   end
end

end

%% use "Greedy approach" to smooth the path we find
function path_smooth = smooth(map, path,nodes)

path_smooth =path(end);
tmp_point = nodes(1,:);
while true
    l_p = length(path);
    for i=1:l_p
        vec = nodes( path(i),:) - tmp_point;
        vec_dir = vec/norm(vec);
        or_reduce = is_add_in_veritces(map ,nodes(path(i),: ),tmp_point,vec_dir,60);
        if or_reduce==1 
           path_smooth = [path_smooth, path(i)];
           tmp_point = nodes(path(i),: );
           break;
        else
            continue;
        end
    end
    vec_goal = nodes(end,:) - tmp_point;
    goal_dir = vec_goal/norm(vec_goal);
    or_goal = is_add_in_veritces(map , nodes(end,: ),tmp_point,goal_dir,60);
    if or_goal==1  
        path_smooth = [path_smooth, path(1)];
        break;
    else
        ind_path = find(path==path(i));
        path=path(1:ind_path);
    end
end

end