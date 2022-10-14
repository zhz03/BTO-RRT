clear all
figure(3);
%ptCloud = pcread('test-1pct.ply');
ptCloud = pcread('box-1pct.ply');
%ptCloud = pcread('hot-1pct.ply');
pcshow(ptCloud);
hold on;
q_init = [175,120,100,0,0,0];
q_init = [230,120,90,0,0,0];
%q_init = [158,150,96,0,0,0];
%q_init = [70,215,110,0,0,0];
q_goal = [70,215,110,1,0,0];
q_goal = [35,20,95,1,0,0];
q_goal = [115,10,95,1,0,0];
q_goal = [90,255,95,1,0,0];
% q_goal = [160,110,93,1,0,0];
%q_goal = [145,185,96,1,0,0];
%%q_goal = [150,30,120,1,0,0];
% q_goal = [175,185,100,1,0,0];
% %new
%  q_init = [10,60,50,0,0,0];
%  q_goal = [10,10,50,1,0,0];
% %ho
% q_init = [50,120,55,0,0,0];
% q_goal = [185,60,30+20,1,0,0];
%hot 
% q_init = [120,190,30+30,0,0,0];
% q_goal = [20,40,40+30,1,0,0];
% % test
% q_init = [10,60,25,0,0,0];
% q_goal = [50,10,25,1,0,0];

plot3(q_init(:,1),q_init(:,2),q_init(:,3),'*b','LineWidth',6);
plot3(q_goal(:,1),q_goal(:,2),q_goal(:,3),'*r','LineWidth',6);
zlabel('z');
ylabel('y');
xlabel('x');
hold on
sizemax.x = ceil(double(ptCloud.XLimits(2))); 
sizemax.y = ceil(double(ptCloud.YLimits(2)));
sizemax.h = ceil(double(ptCloud.ZLimits(2)+50));
%initial conditions
stepsize =10;
throushold = 5;

RRTnode1 = q_init;
RRTnode2 = q_goal;

%% main
tic;
if ( (norm(q_init(1:3)-q_goal(1:3))<throushold)&&(Collisiondetect(q_init,q_goal,ptCloud,sizemax)==0) )
  path = [q_init; q_goal];
else 
%     pathFound1 = 0;
%     pathFound2 = 0;
   pathFound = 0;
while pathFound < 1 %pathFound1<1 || pathFound2<1
%   while pathFound1 <1
%       [RRTnode1,flag1] = extendTree(RRTnode1,RRTnode2,q_goal,stepsize,ptCloud,sizemax);
%       plot3(RRTnode1(:,1),RRTnode1(:,2),RRTnode1(:,3),'*bl');
%       drawnow
%       pathFound1 = pathFound1 + flag1;
% %   end
% %   while pathFound2 <1
%       [RRTnode2,flag2] = extendTree(RRTnode2,RRTnode1,q_init,stepsize,ptCloud,sizemax);
%       plot3(RRTnode2(:,1),RRTnode2(:,2),RRTnode2(:,3),'*r');
%       drawnow
%       pathFound2 = pathFound2 + flag2;
 %   end
 [RRTnode1,flag1] = extendTree(RRTnode1,q_goal,stepsize,ptCloud,sizemax);
    plot3(RRTnode1(:,1),RRTnode1(:,2),RRTnode1(:,3),'*bl');
      drawnow
     % q_goal = RRTnode2(end,:);
 %%%%%%%%%%%%
%       [RRTnode2,flag1] = extendTree(RRTnode2,q_init,stepsize,ptCloud,sizemax);  
%       plot3(RRTnode2(:,1),RRTnode2(:,2),RRTnode2(:,3),'*r');
%   drawnow
%   [RRTnode3,flag2] = extendTree(RRTnode2(end,:),RRTnode1(end,:),stepsize,ptCloud,sizemax);
%   pathFound = pathFound + flag2;
%       %%%%%%%%%%%%%%%%%%
 [RRTnode2,flag2] = extendTree(RRTnode2,RRTnode1(end,:),stepsize,ptCloud,sizemax);
  plot3(RRTnode2(:,1),RRTnode2(:,2),RRTnode2(:,3),'*r');
  drawnow
  pathFound = pathFound + flag2;
end
end
RRTnode1(end,4)=1;
RRTnode2(1,4) = 0;
path1 = findMinimumPath(RRTnode1);
path2 = findMinimumPath(RRTnode2);
%%%%%%%%%%%%%%%%%%%%%%%
path = path1;
path1_l = length(path1(:,1));
path2_l = length(path2(:,1));
for i = path2_l:-1:1
   path = [path; path2(i,:)] ;
end

plot3(path(:,1),path(:,2),path(:,3),'r','LineWidth',2);
% optimize
path_l = length(path(:,1));
dis_path = zeros(path_l,1);
path_start = [];
indx = [0];
path_start = path (1,1:3);
for i=2 : path_l
    dis_path(i) = pdist2(path_start,path(i,1:3));
    if (dis_path(i) > dis_path(i-1)) || (dis_path(i) > 0)
        if Collisiondetect(path_start, path(i,1:3),ptCloud,sizemax) == 1
            path_start = path(i-1,1:3);
            indx = [indx;i-1];
        else continue;
        end
    end    
end
indx = [indx;path_l];
path_opt = path(1,1:3);
for i = 2: length(indx)
    path_opt = [path_opt;path(indx(i),1:3)];
end
plot3(path_opt(:,1),path_opt(:,2),path_opt(:,3),'-y','LineWidth',3);
%insert point
%path_optin = insertp(path_opt,stepsize);
%up sample
if (~isempty(path_opt))
    P = upsample(path_opt,ptCloud,sizemax);
end
plot3(P(1,:),P(2,:),P(3,:),'-g','LineWidth',3);
% B-spline curve
smooth = spcrv([[path_opt(1,1) path_opt(:,1)' path_opt(end,1)];[path_opt(1,2) path_opt(:,2)' path_opt(end,2)];[path_opt(1,3) path_opt(:,3)' path_opt(end,3)]],3); 
%smooth = spcrv([[path(1,1) path(:,1)' path(end,1)];[path(1,2) path(:,2)' path(end,2)];[path(1,3) path(:,3)' path(end,3)]],3); 
smooth_inv = smooth';
sm_len = length(smooth_inv(:,1));
count = [];
for i = 1:sm_len
    safe_dist = 1;
    if checkpoint(smooth_inv(i,:),ptCloud,safe_dist)
       count = [count;i]; 
    end
end

if length(count)
   path_opt_len = length(path_opt(:,1));
   crash_dist = zeros(path_opt_len,1);
   count_len = length(count);
   count_inx = zeros(count_len,1);
   for j =1: count_len
   for i = 1: path_opt_len
       carsh_dist(i) = pdist2(path_opt(i,:),smooth_inv(count(1),:));
   end
   [Amin,count_inx(i)] = min(carsh_dist);
   end
end

plot3(smooth(1,:),smooth(2,:),smooth(3,:), '-b','LineWidth',2); 
t_elapse = toc;
%  RRT distance calculation
% l_path = length(path(:,1));
% dis_path = zeros(l_path-1,1);
% for i=1:l_path - 1
%     dis_path(i) = pdist2(path(i),path(i+1));
% end
%    dis_path_sum = sum(dis_path);
   dis_path_sum = dist_cal(path);
   dis_path_opt_sum = dist_cal(path_opt);
   dis_path_smooth_sum = dist_cal(smooth_inv);
   fprintf('\nTime elapse=%f \n',t_elapse);
   fprintf('Path Original Length = %.3f \n\n',dis_path_sum);
   fprintf('Path Greedy Optimal Length = %.3f \n\n',dis_path_opt_sum);
   fprintf('Path Optimal & smooth Length = %.3f \n\n',dis_path_smooth_sum);
%% function : check points
function node_check = checkpoint(point,ptCloud,safe_dist)
K = 10; % check nearest 10 points
node_check = 0; %no interaction with point cloud
[A,dists] = findNearestNeighbors(ptCloud,point,K);
for i=1:K
    if dists(i) <= safe_dist
        node_check = 1; % it is true that the current point may hit obs.
    end
end
end

%% function 2: Collision detection
function collision_flag = Collisiondetect(node1, node2,ptCloud,sizemax)
safe_dist =3;
collision_flag = 0;
if ((node1(1)>sizemax.x)| (node1(1)<0)| (node1(2)>sizemax.y)| (node1(2)<0))|(node1(3) > sizemax.h) | (node1(3) < 90)
  collision_flag = 1;
else
     for sigma = 0:0.1:1
         p = sigma*node1(1:3) + (1-sigma)*node2(1:3);
         
        if checkpoint(p,ptCloud,safe_dist)
            collision_flag = 1;
        end
     end
end
end

%% function 3: Bi-ExtendTree
function [RRTnode,flag] = extendTree(RRTnode,goal,stepsize,ptCloud,sizemax)
  flag1 = 0;
  qet=1;

 % num = length(ptCloud.x0);
  
  while flag1==0,
    if qet==0
        q_rand = [randi(sizemax.x),randi(sizemax.y),randi(sizemax.h)];
    else
        q_rand = [goal(:,1),goal(:,2),goal(:,3)];
    end
    tmp = sqrt(RRTnode(:,1:3)-ones(size(RRTnode,1),1)*q_rand(:,1:3));
    [dist,idx] = min(diag(tmp*tmp'));
    q_new = (q_rand-RRTnode(idx,1:3));
    q_new = RRTnode(idx,1:3)+q_new/norm(q_new)*stepsize;
    new_node = [q_new, 0, 0, idx];
    if Collisiondetect(new_node, RRTnode(idx,:),ptCloud,sizemax)==0
      RRTnode = [RRTnode; new_node];
      flag1=1;
    else
        qet=0;
        flag1=0;
    end
  end
  % check to see if new node connects directly to end_node
  if ( (norm(new_node(1:3)-goal(1:3))<stepsize )&&(Collisiondetect(new_node,goal,ptCloud,sizemax)==0) )
    flag = 1;
    RRTnode(end,4)=1;  % mark node as connecting to end. end
  else
    flag = 0;
  end 
end
% function [RRTnode1,flag] = extendTree(RRTnode1,RRTnode2,goal,stepsize,ptCloud,sizemax)
%   flag1 = 0;
%   qet=1;
% while flag1 == 0
%     if qet ==0
%         q_rand = [randi(sizemax.x),randi(sizemax.y),randi(sizemax.h)];
%     else
%         q_rand = [goal(:,1),goal(:,2),goal(:,3)];
%     end
%     tmp = sqrt(RRTnode1(:,1:3)-ones(size(RRTnode1,1),1)*q_rand(:,1:3));
%     [dist,idx] = min(diag(tmp*tmp'));
%     q_new = (q_rand-RRTnode1(idx,1:3));
%     q_new = RRTnode1(idx,1:3)+q_new/norm(q_new)*stepsize;
%     new_node = [q_new, 0, 0, idx];
%     if Collisiondetect(new_node, RRTnode1(idx,:),ptCloud,sizemax)==0
%       %new_tree = [RRTnode; new_node];
%       RRTnode1 = [RRTnode1; new_node];
%       flag1=1;
%     else
%         qet=0;
%         flag1=0;
%     end
%     tmp2 = sqrt(RRTnode2(:,1:3)-ones(size(RRTnode2,1),1)*q_rand(:,1:3));
%     [dist2,idx2] = min(diag(tmp2*tmp2'));
% %     q_new2 = (q_rand-RRTnode2(idx,1:3));
% %     q_new2 = RRTnode2(idx,1:3)+q_new2/norm(q_new2)*stepsize;
%     if norm(RRTnode2(:,1:3) - q_new) < stepsize
%        flag = 1;
%        flag1 = 1;
%        RRTnode1(end,4) =1;
%     else 
%        flag = 0;
%     end
%     if ( (norm(new_node(1:3)-goal(1:3))<stepsize )&&(Collisiondetect(new_node,goal,ptCloud,sizemax)==0) )
%     flag = 1;
%     RRTnode1(end,4)=1;  % mark node as connecting to end. end
%      else
%     flag = 0;
%     end  
% end     
% end

%% function 4:

function path = findMinimumPath(tree)
     connectingNodes = [];
    for i=1:size(tree,1),
        if tree(i,4)==1,
            connectingNodes = [connectingNodes; tree(i,:)];
        end
    end
    % find minimum cost last node
    [tmp,idx] = min(connectingNodes(:,5));
    % construct lowest cost path
    path = [connectingNodes(idx,:)];
    parent_node = connectingNodes(idx,6);
   while parent_node>0,
        path = [tree(parent_node,:); path];
        parent_node = tree(parent_node,6);
    end

end 
 
%% function 5: distance calculation
function dis_path_sum = dist_cal(path)
l_path = length(path(:,1));
dis_path = zeros(l_path-1,1);
for i=1:l_path - 1
    dis_path(i) = pdist2(path(i),path(i+1));
end
   dis_path_sum = sum(dis_path);
end
%% function 6: insert point (useless)
function path = insertp(path_opt,stepsize)
path = path_opt';
[nn,m] = size(path_opt);
clearvars m;
len = zeros(nn-1,1);
num = len;
for  k = 2:nn
    len(k) = norm(path(:,k)-path(:,k-1));
    num(k) = floor(len(k)/stepsize) + 1;
end
px = [];
py = [];
pz =[];
for  k = 2:nn
    tx = linspace(path(1,k-1),path(1,k),num(k));
    ty = linspace(path(2,k-1),path(2,k),num(k));
    tz = linspace(path(3,k-1),path(3,k),num(k));
    px = [px(1:end-1),tx]; py = [py(1:end-1),ty]; pz = [pz(1:end-1),tz]; 
end
path = [px;py;pz];
end
%% function 7: upsample
function P = upsample(path,ptCloud,sizemax)
P = path';
[n m] =size(P);
clearvars n;
l = zeros(m,1);
for k =2:m
    l(k) = norm(P(:,k)-P(:,k-1)) + l(k-1);
end
l_init = l(m);
iter =1;
while iter <= 1000;
    s1 = rand(1,1)*l(m); 
    s2 = rand(1,1)*l(m);  
     if s2 < s1
        temps = s1;
        s1 = s2;
        s2 = temps;
     end
    for k=2:m
        if s1 < l(k)
            i = k - 1;
            break;
        end
    end
        for k=(i+1):m
        if s2 < l(k)
            j = k - 1;
            break;
        end
    end
    if (j <= i)
        iter = iter + 1;
        continue;
    end
    t1 = (s1 - l(i))/(l(i+1)-l(i));
    gamma1 = (1 - t1)*P(:,i) + t1*P(:,i+1);
    t2 = (s2 - l(j))/(l(j+1)-l(j));
    gamma2 = (1 - t2)*P(:,j) + t2*P(:,j+1);
    col1 = Collisiondetect(P(:,i)', gamma1',ptCloud,sizemax);
    if col1 == 1
        iter = iter + 1;
        continue;
    end
    col2 = Collisiondetect(gamma1', gamma2',ptCloud,sizemax);
    if col2 == 1
        iter = iter + 1;
        continue;
    end
    col3 = Collisiondetect(P(:,j)', gamma2',ptCloud,sizemax);
    if col3 == 1
        iter = iter + 1;
        continue;
    end
        col4 = Collisiondetect(P(:,j)', P(:,i)',ptCloud,sizemax);
    if col4 == 1
        iter = iter + 1;
        continue;
    end
    newP = [P(:,1:i) gamma1 gamma2 P(:,j+1:m)];
    clearvars P;
    P = newP;
    [n,m] = size(P);
    clearvars n;
    l = zeros(m,1);
    for k=2:m
        l(k)=norm(P(:,k)-P(:,k-1)) + l(k-1);
    end
    iter = iter + 1;
end
end
