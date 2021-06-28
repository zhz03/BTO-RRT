
clc
clear all

tic;

% data.h =[ 5,35,25,38,30,10,50,30,15,45,34,40,20,20];
% data.x0=[10,40,45,60,20,20,50,20,30,70,80,25,15,40];
% data.y0=[10,25,50,30,45,10,60,60,10,60,70,80,30,10];
% num = length(data.x0);
% data.xi=[5.5,8,5,4.5,5.5,3.5,5,5,5,8,20,10,8,10];
% data.yi=[5,7,6,5.5,6,4.5,5,8,5,7,8,15,10,10];
% sizemax = 100;
% Z2=CeatHill(num,data.h,data.x0,data.y0,data.xi,data.yi,sizemax); 
data.h =[ 5,35,25,38,30,10,50,30,15,45,34,40,20,20,25,30,-10,25];
data.x0=[10,40,45,60,20,20,50,20,30,70,80,25,15,40,50,70,90,110];
data.y0=[10,25,50,30,45,10,60,60,10,60,70,80,30,10,10,10,40,40];
num = length(data.x0);
data.xi=[5.5,8,5  ,4.5  ,5.5,3.5,5,5,5,8,20,10,8,10,10,10,20,10];
data.yi=[5,7,6,5.5,6,4.5,5  ,8  ,5,7,8,15,10,10,10,10,20,30]; 
sizemax = 130;
Z2=CeatHill(num,data.h,data.x0,data.y0,data.xi,data.yi,sizemax); 
figure(1);
ylabel('y');
xlabel('x');
surf(Z2); 
shading flat;
segmentLength =6;
start_node =  [0,0,5,0,0,0];
end_node   =[50,20,20,1,0,0];
end_node   =[10,90,44,1,0,0];
hold on
plot3(start_node(:,1),start_node(:,2),start_node(:,3),'*b','LineWidth',6);
plot3(end_node(:,1),end_node(:,2),end_node(:,3),'*r');
tree = start_node;
if ( (norm(start_node(1:3)-end_node(1:3))<segmentLength )&&(collision(num,start_node,end_node,Z2,data,sizemax)==0) )
  path = [start_node; end_node];
  else
  numPaths = 0;
  while numPaths<1,
      [tree,flag] = extendTree(tree,end_node,segmentLength,Z2,data,sizemax);
      plot3(tree(:,1),tree(:,2),tree(:,3),'*g');
      drawnow
      numPaths = numPaths + flag;
  end
end
 path = findMinimumPath(tree);
 path = [start_node;path];
 path = [path;end_node];
 plot3(path(:,1),path(:,2),path(:,3),'r','LineWidth',2);  
 toc;
function [data]=CeatHill(N,h,x0,y0,xi,yi,num) 
x=1:1:num;y=1:1:num;
for m=1:num
    for n=1:num
        Sum=0;
        for k=1:N
           s=h(k)*exp(-((x(m)-x0(k))/xi(k))^2-((y(n)-y0(k))/yi(k))^2);
           Sum=Sum+s;
        end
        data(m,n)=Sum;
    end
end

end
function collision_flag = collision(N,node, parent,Z2,data,num)
collision_flag = 0;
%  h=[20,35,25,38,20,25];
% x0=[10,40,45,60,20,20];
% y0=[10,25,50,30,45,10];
% xi=[5.5,8,5,4.5,5.5,3.5];
% yi=[5,7,6,5.5,6,4.5];
Z1=Z2;
% for i=1:80
%     for j=1:80
%         if(Z1(j,i)>high)
%             Z1(j,i)=10000;
%         end
%     end
% end
if ((node(1)>num)| (node(1)<0)| (node(2)>num)| (node(2)<0))
  collision_flag = 1;
else
     for sigma = 0:0.1:3,
         p = sigma*node(1:3) + (1-sigma)*parent(1:3);
          Sum1=0;
        for k=1:N
           s=data.h(k)*exp(-((p(2)-data.x0(k))/data.xi(k))^2-((p(1)-data.y0(k))/data.yi(k))^2);
           Sum1=Sum1+s;
        end
        if(p(3)<Sum1)
            collision_flag = 1;
        end
     end
end
end
function [new_tree,flag,high] = extendTree(tree,end_node,segmentLength,Z2,data,sizemax)
  flag1 = 0;
  qet=1;
  prb = 0.8;
  num = length(data.x0);
  while flag1==0,
    % select a random point
    if (qet==0)%rand < prb
        randomPoint = [randi(sizemax),randi(sizemax),randi(sizemax)];
        %randomPoint = [sizemax*rand,sizemax*rand,sizemax*rand];
    else
        randomPoint = [end_node(:,1),end_node(:,2),end_node(:,3)];
    end
    % find leaf on node that is closest to randomPoint
    %0.45*sqrt(tree(:,1:2)-ones(size(tree,1),1)*randomPoint)
    
    %tmp = sqrt(tree(:,1:3)-ones(size(tree,1),1)*end_node(:,1:3));
    tmp = sqrt(tree(:,1:3)-ones(size(tree,1),1)*randomPoint(:,1:3));
    [dist,idx] = min(diag(tmp*tmp'));
     A=end_node(:,1:2)-tree(idx,1:2);
     B=randomPoint(:,1:2)-tree(idx,1:2);
    agl=acos((dot(A,B)/(norm(A)*norm(B))))*180/pi;
   if (agl>sizemax)
       continue;
   end
    cost= tree(idx,4) + segmentLength;
    new_point = (randomPoint-tree(idx,1:3));
    new_point = tree(idx,1:3)+new_point/norm(new_point)*segmentLength;
    if(qet==1)
        high=new_point(:,3);
    end
    if(tree(idx,3)==end_node(:,3))
        new_point(:,3)=end_node(:,3);
    end
    if(tree(idx,3)>end_node(:,3))
    new_point(:,3)=tree(idx,3)-new_point(:,3)/norm(new_point)*segmentLength;
    end
    new_node = [new_point, 0, cost, idx];
%     hold on
    if collision(num,new_node, tree(idx,:),Z2,data,sizemax)==0,
      new_tree = [tree; new_node];
      %plot3(new_point(:,1),new_point(:,2),new_point(:,3),'r*');
      flag1=1;
    else
            qet=0;
            flag1=0;
    end
  end
  % check to see if new node connects directly to end_node
  if ( (norm(new_node(1:3)-end_node(1:3))<segmentLength )&&(collision(num,new_node,end_node,Z2,data,sizemax)==0) )
    flag = 1;
    new_tree(end,4)=1;  % mark node as connecting to end. endÃ»ÕÒµ½
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
    
   while parent_node>1,
        path = [tree(parent_node,:); path];
        parent_node = tree(parent_node,6);

    end

 end  
 