% have passed in diff maps
clear all
figure(4);
%ptCloud = pcread('box-1pct.ply');
ptCloud = pcread('hot-1pct.ply');
%ptCloud = pcread('test-1pct.ply');
pcshow(ptCloud);
hold on;
q_init = [175,120,100,0,0,0];
%q_goal = [70,215,130,1,0,0];
% q_goal = [175,120,95,1,0,0];
q_init = [70,215,100,0,0,0];
q_goal = [35,20,120,1,0,0];
q_goal = [35,200,120,1,0,0];
%hot
q_init = [50,120,25,0,0,0];
q_goal = [185,60,30,1,0,0];
q_init = [120,190,30,0,0,0];
q_goal = [20,40,40,1,0,0];
% test
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
sizemax.h = ceil(double(ptCloud.ZLimits(2)+20));

%initial conditions
stepsize =2;
throushold = 0.5;
RRTnode = q_init;
%% main
%check = checkpoint(q_init,double(ptCloud.Location))
tic;
if ( (norm(q_init(1:3)-q_goal(1:3))<throushold)&&(Collisiondetect(q_init,q_goal,ptCloud,sizemax)==0) )
  path = [q_init; q_goal];
  else
  pathFound = 0;
  while pathFound<1,
      [RRTnode,flag] = extendTree(RRTnode,q_goal,stepsize,ptCloud,sizemax);
      plot3(RRTnode(:,1),RRTnode(:,2),RRTnode(:,3),'*bl');
      drawnow
      pathFound = pathFound + flag;
  end
end
path = findMinimumPath(RRTnode);
path = [path;q_goal];
plot3(path(:,1),path(:,2),path(:,3),'r','LineWidth',2);  
% optimize
path_l = length(path(:,1));
dis_opt = zeros(path_l,1);
path_start = [];
indx = [0];
path_start = path (1,1:3);
for i=2 : path_l
    dis_opt(i) = pdist2(path_start,path(i,1:3));
    if (dis_opt(i) > dis_opt(i-1)) || (dis_opt(i) > 0)
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
%
smooth = spcrv([[path_opt(1,1) path_opt(:,1)' path_opt(end,1)];[path_opt(1,2) path_opt(:,2)' path_opt(end,2)];[path_opt(1,3) path_opt(:,3)' path_opt(end,3)]],3); 
%smooth = spcrv([[path(1,1) path(:,1)' path(end,1)];[path(1,2) path(:,2)' path(end,2)];[path(1,3) path(:,3)' path(end,3)]],3); 
plot3(smooth(1,:),smooth(2,:),smooth(3,:), '-g','LineWidth',2); 
t_elapse = toc;
% distance calculation
l_opt = length(path(:,1));
dis_opt = zeros(l_opt-1,1);
for i=1:l_opt - 1
    dis_opt(i) = pdist2(path(i),path(i+1));
end
   dis_sum = sum(dis_opt);
   fprintf('\nTime elapse=%f \n',t_elapse);
   fprintf('Path Optimal Length = %.3f \n\n',dis_sum);

%% function 1: check point 
function node_check = checkpoint(point,ptCloud)
safe_dist = 3;
%size = length(ptCloud_Location(:,1));
%                                   
K = 10;
node_check = 0; %no interset with point cloud
[indices,dists] = findNearestNeighbors(ptCloud,point,K);
for i=1:K
    if dists(i) <= safe_dist
        node_check = 1; 
    end
end
end
%% function 2: collision detection
function collision_flag = Collisiondetect(node1, node2,ptCloud,sizemax)
collision_flag = 0;
if ((node1(1)>sizemax.x)| (node1(1)<0)| (node1(2)>sizemax.y)| (node1(2)<0))
  collision_flag = 1;
else
     for sigma = 0:0.1:1
         p = sigma*node1(1:3) + (1-sigma)*node2(1:3);
         
        if checkpoint(p,ptCloud)
            collision_flag = 1;
        end
     end
end
end

%% function 3
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
    % find leaf on node that is closest to randomPoin
%     if rand < prb
% tmp = sqrt(RRTnode(:,1:3)-ones(size(RRTnode,1),1)*goal(:,1:3));
%     else
%     tmp = sqrt(RRTnode(:,1:3)-ones(size(RRTnode,1),1)*q_rand(:,1:3));
%     end
    %tmp = 0.5*sqrt(RRTnode(:,1:3)-ones(size(RRTnode,1),1)*goal(:,1:3))+0.5* sqrt(RRTnode(:,1:3)-ones(size(RRTnode,1),1)*q_rand(:,1:3));
    tmp = sqrt(RRTnode(:,1:3)-ones(size(RRTnode,1),1)*q_rand(:,1:3));
    [dist,idx] = min(diag(tmp*tmp'));
    q_new = (q_rand-RRTnode(idx,1:3));
    q_new = RRTnode(idx,1:3)+q_new/norm(q_new)*stepsize;
%     if(RRTnode(idx,3)==goal(:,3))
%         q_new(:,3)=goal(:,3);
%     end
%     if(RRTnode(idx,3)>goal(:,3))
%     q_new(:,3)=RRTnode(idx,3)-q_new(:,3)/norm(q_new)*segmentLength;
%     end
%     if(RRTnode(idx,3)<goal(:,3))
%     q_new(:,3)=RRTnode(idx,3)+q_new(:,3)/norm(q_new)*segmentLength;
%     end
    new_node = [q_new, 0, 0, idx];
    if Collisiondetect(new_node, RRTnode(idx,:),ptCloud,sizemax)==0
      %new_tree = [RRTnode; new_node];
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
%% function 4
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
 