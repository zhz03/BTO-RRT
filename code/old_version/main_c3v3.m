
clear all
% environment1 data
data.h =[ 5,35,25,38,30,10,50,30,15,45,34,40,20,20,25,30,-10,25];
data.x0=[10,40,45,60,20,20,50,20,30,70,80,25,15,40,50,70,90,110];
data.y0=[10,25,50,30,45,10,60,60,10,60,70,80,30,10,10,10,40,40];
num = length(data.x0);
data.xi=[5.5,8,5  ,4.5  ,5.5,3.5,5,5,5,8,20,10,8,10,10,10,20,10];
data.yi=[5,7,6,5.5,6,4.5,5  ,8  ,5,7,8,15,10,10,10,10,20,30]; 


%enviroment data2
% N = 20;
% data.h =  [10,20,30,40,50,10,20,30,40,50,10,20,30,40,50,10,50,10,20,30,40,50,10,40,50,10,20,60,30,30];
% data.x0 = [20,20,20,20,20,20,20,20,20,40,40,40,40,40,40,40,40,40,60,60,60,60,60,60,60,60,60,160,100,100];
% data.y0 = [20,40,60,80,100,120,140,160,180,20,40,60,80,100,120,140,160,180,20,40,60,80,100,120,140,160,180,120,40,160];
% data.xi = 5 .*ones(27,1);
% data.yi = 5 .*ones(27,1);
% data.xi = [data.xi;40;20;30];
% data.yi = [data.yi;40;30;20];
% sizemax.x = 200;
% sizemax.y = 200;
sizemax.x = 130; 
sizemax.y = 130;
sizemax.z = max(data.h);
Enxyz=CreatEnxyz(data,sizemax); 
% plot Enxyz
figure(1);
surf(Enxyz); 
shading flat;
zlabel('z');
ylabel('y');
xlabel('x');
% task setting
q_init =  [0,0,35,0,0,0];
q_goal = [90,90,50,1,0,0];
q_goal = [10,92,10,1,0,0];
hold on
plot3(q_init(:,1),q_init(:,2),q_init(:,3),'*b','LineWidth',6);
plot3(q_goal(:,1),q_goal(:,2),q_goal(:,3),'*r','LineWidth',6);
%initial conditions
stepsize =10;
RRTnode = q_init;
%% main
tic;
if ( (norm(q_init(1:3)-q_goal(1:3))<stepsize )&&(Collisiondetect(q_init,q_goal,data,sizemax)==0) )
  path = [q_init; q_goal];
  else
  pathFound = 0;
  while pathFound<1,
      [RRTnode,flag] = extendTree(RRTnode,q_goal,stepsize,data,sizemax);
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
        if Collisiondetect(path_start, path(i,1:3),data,sizemax) == 1
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
   
 %% function part1: CreatEnxyz 
function Enxyz = CreatEnxyz(data,sizemax) 
xmax = sizemax.x; ymax = sizemax.y;
x=1:1:xmax;y=1:1:ymax;
num = length(data.x0);
for m=1:xmax
    for n=1:ymax
        Sum=0;
        for k=1:num
           s=data.h(k)*exp(-((x(m)-data.x0(k))/data.xi(k))^2-((y(n)-data.y0(k))/data.yi(k))^2);
           Sum=Sum+s;
        end
        Enxyz(m,n)=Sum;
    end
end

end
%% function part2:Collisiondetect 
function collision_flag = Collisiondetect(node1, node2,data,sizemax)
collision_flag = 0;
num = length(data.x0);
if ((node1(1)>sizemax.x)| (node1(1)<0)| (node1(2)>sizemax.y)| (node1(2)<0))
  collision_flag = 1;
else
     for sigma = 0:0.1:1
         p = sigma*node1(1:3) + (1-sigma)*node2(1:3);
          Sum1=0;
        for k=1:num
           s=data.h(k)*exp(-((p(2)-data.x0(k))/data.xi(k))^2-((p(1)-data.y0(k))/data.yi(k))^2);
           Sum1=Sum1+s;
        end
        if(p(3)<(Sum1+5))
            collision_flag = 1;
        end
     end
end
end
%% function part3: classiRRT
function [RRTnode,flag] = extendTree(RRTnode,goal,stepsize,data,sizemax)
  flag1 = 0;
  qet=1;

  num = length(data.x0);
  
  while flag1==0,
    if qet==0
        q_rand = [randi(sizemax.x),randi(sizemax.y),randi(sizemax.z)];
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
    if Collisiondetect(new_node, RRTnode(idx,:),data,sizemax)==0,
      %new_tree = [RRTnode; new_node];
      RRTnode = [RRTnode; new_node];
      flag1=1;
    else
        qet=0;
        flag1=0;
    end
  end
  % check to see if new node connects directly to end_node
  if ( (norm(new_node(1:3)-goal(1:3))<stepsize )&&(Collisiondetect(new_node,goal,data,sizemax)==0) )
    flag = 1;
    RRTnode(end,4)=1;  % mark node as connecting to end. end
  else
    flag = 0;
  end 
end
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
 