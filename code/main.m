
clear 
close all
%load map.mat map
I = imread('map1.bmp');
%I = imread('warran.png');
n = graythresh(I);
mm = im2bw(I,n);
imshow(mm);
MM = double(mm);
save MM.mat MM
imshow(MM);
[a,b]=size(MM);
for i = 1:a
    for j= 1:b
        if MM(i,j) == 1
            MM(i,j) = 0;
        else MM(i,j) = 1;
        end
    end
end

map = MM;
[map_height,map_width]=size(map); %row is y?column is x

q_start = [297,740]; % start from Atkinson Hall
q_goal = [ 966 300];  % end at EBU II
q_start=[490 490]; % source position in Y, X format
q_goal=[46 85]; % goal position in Y, X format
 q_start=[297 746]; % In Y, X format
 q_goal=[1045 202];
 q_start=[490 490]; % In Y, X format
 q_goal=[46 85];
colormap=[1 1 1
          0 0 0
          1 0 0
          0 1 0
          0 0 1];
imshow(uint8(map),colormap)
hold on

tic;  % tic-toc: Functions for Elapsed Time
%initial condition
vertices=q_start;
edges = [];
K=10000;
delta_q=20;
p=0.5;
q_rand=[];
q_near=[];
q_new=[];
%main function
plot(q_start(2),q_start(1),'*b','LineWidth',4)
plot(q_goal(2),q_goal(1),'*r', 'LineWidth', 4)
for k = 1:K
    arrived=is_goal_arrived(vertices,q_goal,delta_q);
    if arrived
        vertices=[vertices;q_goal];
        edges = [edges;[size(vertices,1),size(vertices,1)-1]];
        break;
    end
    if rand <= p
        q_rand = q_goal;%q(1)width: x?q(2)height:y
    else
        q_rand = [randi(map_height),randi(map_width)];
    end
    if map( q_rand(1,1),q_rand(1,2) ) == 1 %map(1)height,map(2)width
        continue;
    end
    [q_new,q_near,q_near_ind,vector_dir] = get_qnew_qnear(delta_q,q_rand,vertices);
    add_qnew = is_add_in_veritces(map,q_new,q_near,vector_dir,10);
    if add_qnew
        vertices=[vertices;q_new];
        r_v = size(vertices,1);
        edges = [edges;[r_v,q_near_ind]];
    else
        continue;
    end
%     plot(q_near(1,1),q_near(2,1),'*b');
   plot([q_near(1,2),q_new(1,2)],[q_near(1,1),q_new(1,1)],'-b')
   drawnow
end
path =find_path_node(edges);
%plot base path
plot(vertices(path,2),vertices(path,1),'-r')
%smooth
path_smooth = smooth(path,vertices,map);
%plot smooth path
plot(vertices(path_smooth,2),vertices(path_smooth,1),'g','LineWidth', 2);

%calculate the cumulative distance
disp = [vertices(path_smooth,2),vertices(path_smooth,1)];
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


function arrived=is_goal_arrived(vertices,q_goal,delta_q)

dist=pdist2(vertices(end,:),q_goal);
if dist <= delta_q
    arrived=1;
else
    arrived=0;
end
end

function [q_new,q_near,q_near_ind,vector_dir] = get_qnew_qnear(delta_q,q_rand,vertices)

dist_rand = pdist2(vertices,q_rand);
[dist_min,q_near_ind]=min(dist_rand);
q_near=vertices(q_near_ind,:);
vector_dir =q_rand-q_near;
vector_dir = vector_dir./dist_min;
if dist_min > delta_q    
    q_new = floor( q_near+delta_q*vector_dir );
else
    q_new=q_rand;
end
end

function add_qnew = is_add_in_veritces(map,q_new,q_near,vector_dir,insert_p)

dist_new2near = norm(q_new - q_near);
dist_gap = dist_new2near/insert_p;
ii =1:insert_p;
insert_point = repmat(q_near,insert_p,1)+ii'.*dist_gap* vector_dir;
insert_point =[floor(insert_point);q_new];
insert_num = sub2ind(size(map),insert_point(:,1),insert_point(:,2));
or =find( map(insert_num)==1 );
if ~isempty(or)
    add_qnew=0;
else
    add_qnew=1;
end

end

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

