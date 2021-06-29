clear all
%% load data and initialize conditions

map=im2bw(imread('maze.bmp')); % input map read from a bmp file. for new maps write the file name here
map = im2bw(imread('map3.bmp'));
%map=im2bw(imread('map3.png'));
map = im2bw(imread('warran.png'));
%map = im2bw(imread('maze1.jpg'));
%mapd = double(map);
[map_h,map_w]=size(map); 
%task points
q_init=[10 10]; % source position in Y, X format
q_goal=[490 490]; % goal position in Y, X mapformat
%maze
% q_init=[490 490]; % source position in Y, X format
% q_goal=[193 140]; % goal position in Y, X format
%q_goal=[11 113];  % map test
%map3
% q_goal=[361 333];
% q_goal=[329 287];
%jacobs
%  q_init=[297 746]; % In Y, X format
%  q_goal=[819 307];
q_init=[303 747]; % source position in Y, X format
q_goal=[971 429]; % goal position in Y, X format
q_goal = [1063 820];
q_goal = [92 294];
%maze 1
% q_init=[1033 27]; % In Y, X format
% q_goal=[782 2033];
%initial conditions
delta = 20; 
threshold = 20; 
K = 100000;
k=0;
RRTnode=double([q_init -1]);
pathFound=false;
pb = 0.1;
%% main function
tic;


imshow(map); hold on;
if ~(checkpoint(q_init,map) && checkpoint(q_goal,map)) 
    error('task point error! Reassign the task point'); 
end
plot(q_init(2),q_init(1),'*b','LineWidth',4);
plot(q_goal(2),q_goal(1),'*r', 'LineWidth', 4);

while k<=K  % loop to grow RRTs
    if  rand < pb 
        %q_rand = rand(1,2) .* size(map); 
        q_rand = [randi(map_h),randi(map_w)];% random sample
    else
        q_rand=q_goal; % sample taken as goal to bias tree generation to goal
    end
    [Amin, Indx]=min( pdist2(RRTnode(:,1:2),q_rand) ,[],1); % find closest as per the function in the metric node to the sample
    q_near = RRTnode(Indx,1:2);
    theta=atan2(q_rand(1)-q_near(1),q_rand(2)-q_near(2));  % direction to extend sample to produce new node
    q_new = double(int32(q_near(1:2) + delta * [sin(theta)  cos(theta)]));
    if ~checkpath(q_near(1:2), q_new, map) % if extension of closest node in tree to the new point is feasible
        k=k+1;
        continue;
    end
    if pdist2(q_new,q_goal)<threshold 
        pathFound=true;
        break; 
    end % goal reached
    [Amin, Indx2]=min( pdist2(RRTnode(:,1:2),q_new) ,[],1); % check if new node is not already pre-existing in the tree
    if pdist2(q_new,RRTnode(Indx2,1:2)) < threshold
        k=k+1;
        continue; 
    end 
    k=0;
    RRTnode=[RRTnode;q_new Indx]; % add node
    plot([q_near(2);q_new(2)],[q_near(1);q_new(1)],'-b')
    drawnow
end
if ~pathFound
    error('no path found. maximum attempts reached');
end
%retrieve path
path=[q_goal];
prev=Indx;
while prev>0
    path=[RRTnode(prev,1:2);path];
    prev=RRTnode(prev,3);
end
pathOrigin=0;
for i=1:length(path)-1
    pathOrigin=pathOrigin+pdist2(path(i,1:2),path(i+1,1:2)); 
end
t_elapse = toc;
% plot the successfuk path & nodes
plot (path(:,2),path(:,1),'MarkerEdgeColor','r','LineWidth', 2);
plot(path(:,2),path(:,1),'*r','LineWidth',6)
%plot optimal path
path_opt = optimization(map,path);
path_opt = [path_opt;q_goal];
plot(path_opt(:,2),path_opt(:,1),'-g','LineWidth',2)
% smooth
% smooth = spcrv([[path_opt(1,2) path_opt(:,2)' path_opt(end,2)];[path_opt(1,1) path_opt(:,1)' path_opt(end,1)]],3); 
% plot(smooth(1,:),smooth(2,:), '-g','LineWidth',2); 

% calculate the smooth path length
l_opt = length(path_opt);
dis_opt = zeros(l_opt-1,1);
for i = 1: l_opt-1
    dis_opt(i) = pdist2(path_opt(i),path_opt(i+1));
end
dis_sum = sum(dis_opt);
% Display info we need
fprintf('\nTime elapse=%f \n', t_elapse); 
fprintf('Path Original Length=%.3f \n',pathOrigin);
fprintf('Path Optimal Length = %.3f \n\n',dis_sum);
%% checkpoint funtion 
function check=checkpoint(point,map)
if ~(point(1)>=1 && point(1)<=size(map,1) && point(2)>=1 && point(2)<=size(map,2) && map(point(1),point(2))==1)
    check=false;
else check=true;
end
end

%% checkpath function
function check=checkpath(q_near,q_new,map)
check=true;
theta=atan2(q_new(1)-q_near(1),q_new(2)-q_near(2));
for i=0:1:sqrt(sum((q_near-q_new).^2))
    poscheck=q_near+i.*[sin(theta) cos(theta)];
    if ~(checkpoint(ceil(poscheck),map) && checkpoint(floor(poscheck),map) && checkpoint([ceil(poscheck(1)) floor(poscheck(2))],map) && checkpoint([floor(poscheck(1)) ceil(poscheck(2))],map))
        check=false;
        break;
    end
    if ~checkpoint(q_new,map)
        check=false; 
    end
end
end

%% path optimization
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
        end
    if ~checkpath(path_tem,path_new,map)
        path_tem = [path(indx-1,1) path(indx-1,2)]; 
        path_opt = [path_opt; path_tem];
    end
end

end