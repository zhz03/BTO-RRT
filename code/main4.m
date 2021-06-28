%% load data and initial conditions
map=im2bw(imread('maze.bmp'));
q_ini = [490 490];
q_goal = [925 267];
delta = 30;
threshold = 20;
K = 10000;
state = true;
pathfound = false;
k = 0;
p = 0.5;
q_new = q_ini;
counter = 0;
%% verify our task
if ~checkpoint(map,q_ini) 
    error ('task point error! reassign the task!');
end
if ~checkpoint(map,q_goal)
    error ('task point error! reassign the task!');
end
imshow(map);
%rectangle('position',[1 1 size(map)-1],'edgecolor','k','LineWidth',2);
hold on
plot(q_ini(2),q_ini(1),'*b','LineWidth',4);
plot(q_goal(2),q_goal(1),'*r', 'LineWidth', 4);
%% main function 
tic; % tic-toc: Functions for Elapsed Time
RRTnode=double([q_ini -1]);
% for k =1:K
%     
%     
% end
while k <= K
    if pdist2(q_new,q_goal) < threshold
        pathfound = true;
        break;
    end
    if rand < 0.5, 
        q_rand=rand(1,2) .* size(map); % random sample
    else
        q_rand=q_goal; % sample taken as goal to bias tree generation to goal
    end
    [Amin, Indx]=min( pdist2(RRTnode(:,1:2),q_rand) ,[],1); %??A??dim?????????????
    q_near = RRTnode(Indx(1),1:2);
    theta=atan2(q_rand(1)-q_near(1),q_rand(2)-q_near(2));
    q_new = double(int32(q_near(1:2) + delta * [sin(theta)  cos(theta)]));
    if ~checkline(map,q_near,q_new)
        k = k+1;
        continue;
    end
     % check if new node is not already pre-existing in the tree
     [Amin, Indx2]=min( pdist2(RRTnode(:,1:2),q_new) ,[],1);
    if pdist2(q_new,RRTnode(Indx2(1),1:2)) < threshold
        k = k+1;
        continue;
    end
    RRTnode=[RRTnode;q_new Indx(1)]; % add node
    k = 0;
    if state
        line([q_near(2);q_new(2)],[q_near(1);q_new(1)]);
        counter = counter + 1;
        M(counter) = getframe;
    end
end
if state && pathfound
    line([q_near(2);q_new(2)],[q_near(1);q_new(1)],'MarkerEdgeColor','r');
    counter = counter + 1;
    M(counter) = getframe;
end
if ~pathfound 
    error('no path found. maximum attempts reached'); 
end

t2 =toc;
fprintf('Elapsed Time of Runing the program = %f\n\n',t2);

%% checkpoint function
function check = checkpoint(map,point)
 if ~(point(1)<=size(map,1) && point(2)<=size(map,2) && map(point(1),point(2))==1)
     check = false;
 else check = true;
 end  
end

%% checkline function
function check = checkline(map,q_near,q_new)
theta_dir = atan2 (q_new(1)-q_near(1),q_new(2)-q_near(2));
check = true;
endp = sqrt(sum((q_new - q_near).^2));
for i = 0:1:endp
    pos = q_near + i.*[sin(theta_dir) cos(theta_dir)];
    if ~checkpoint(map,q_new)
        check = false;
    end
    if ~(checkpoint(map,ceil(pos))&& checkpoint(map,floor(pos)) && checkpoint(map,[floor(pos(1)),ceil(pos(1))]) && checkpoint(map,[ceil(pos(1)),floor(pos(1))]))
        check = false;
    end
end
end
