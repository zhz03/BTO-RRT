clear all;
%map=im2bw(imread('map7.png')); % input map read from a bmp file. for new maps write the file name here
%map=im2bw(imread('RRTsmart41.jpg'));
%map=im2bw(imread('maze.bmp'));
%map=im2bw(imread('map3.png'));
map=im2bw(imread('warran.png'));
map=im2bw(imread('EBRRT5.jpg'));
%map = im2bw(imread('map1_c4_2.png'));
%q_init=[10 490]; % source position in Y, X format
%q_init=[317 297];
%q_goal=[490 10]; % goal position in Y, X format
%q_goal =[467 453];% q_goal = [356 487];
%q_goal =[10 10];
%map EBRRT5
q_init=[30 40];
q_goal=[300 400];

%smart41
%q_init=[330 350];
%q_goal=[50 350];

%map 11
%q_init=[150 150];
%q_goal=[350 340]; 
%map10
%q_init=[250 30];
%q_goal=[240 490];
%map 9
%q_init=[150 30];
%q_goal=[240 490];
% map8
%q_init=[40 410];
%q_goal=[300 240];
% map7
%q_init=[50 225];
%q_goal=[490 245];
%map 1-6
%q_init=[10 10];
%q_goal=[490 490];
%maze bug1
% q_init=[490 490]; % source position in Y, X format
% q_goal=[566 123]; % goal position in Y, X format
% %maze bug2
% q_init=[490 490]; % source position in Y, X format
% q_goal=[318 340]; % goal position in Y, X format
%jacob
%q_init=[297 740 ]; % source position in Y, X format
%q_goal=[971 429]; 
%q_goal=[1013 430];% goal position in Y, X format

stepsize=20; % size of each step of the RRT
disTh=20; % nodes closer than this threshold are taken as almost the same
maxFailedAttempts = 10000;


%%%%% parameters end here %%%%%

tic;
figure(2)
imshow(map); hold on;
if ~(checkpoint(q_init,map) && checkpoint(q_goal,map)) 
    error('task point error! Reassign the task point'); 
end
plot(q_init(2),q_init(1),'*b','LineWidth',4);
plot(q_goal(2),q_goal(1),'*r', 'LineWidth', 4);

RRTnode1=double([q_init -1]); % First RRT rooted at the source, representation node and parent index
RRTnode2=double([q_goal -1]); % Second RRT rooted at the goal, representation node and parent index

Expansion_state1=false; % sets to true if expansion after set number of attempts fails
Expansion_state2=false; % sets to true if expansion after set number of attempts fails
while ~Expansion_state1 || ~Expansion_state2  % loop to grow RRTs
    if ~Expansion_state1 
        [RRTnode1,Tree_State,Expansion_state1]=rrtExtend(RRTnode1,RRTnode2,q_goal,stepsize,maxFailedAttempts,disTh,map); % RRT 1 expands from source towards goal
        if ~Expansion_state1 && isempty(Tree_State) %&& display
            plot([RRTnode1(end,2);RRTnode1(RRTnode1(end,3),2)],[RRTnode1(end,1);RRTnode1(RRTnode1(end,3),1)],'color','b');
            drawnow
            %line([RRTree1(end,2);RRTree1(RRTree1(end,3),2)],[RRTree1(end,1);RRTree1(RRTree1(end,3),1)],'color','b');
            %counter=counter+1;M(counter)=getframe;
        end
    end
    if ~Expansion_state2 
        [RRTnode2,Tree_State,Expansion_state2]=rrtExtend(RRTnode2,RRTnode1,q_init,stepsize,maxFailedAttempts,disTh,map); % RRT 1 expands from goal towards source
        if ~isempty(Tree_State), Tree_State(3:4)=Tree_State(4:-1:3); end % path found
        if ~Expansion_state2 && isempty(Tree_State) %&& display
            plot([RRTnode2(end,2);RRTnode2(RRTnode2(end,3),2)],[RRTnode2(end,1);RRTnode2(RRTnode2(end,3),1)],'color','r');
            drawnow
            %line();
            %counter=counter+1;M(counter)=getframe;
        end
    end
    if ~isempty(Tree_State) % path found
         %if display
         plot([RRTnode1(Tree_State(1,3),2);Tree_State(1,2);RRTnode2(Tree_State(1,4),2)],[RRTnode1(Tree_State(1,3),1);Tree_State(1,1);RRTnode2(Tree_State(1,4),1)],'color','green');
         drawnow   
         %line();
            %counter=counter+1;M(counter)=getframe;
        %end
        path=[Tree_State(1,1:2)]; % compute path
        prev=Tree_State(1,3); % add nodes from RRT 1 first
        while prev>0
            path=[RRTnode1(prev,1:2);path];
            prev=RRTnode1(prev,3);
        end
        prev=Tree_State(1,4); % then add nodes from RRT 2
        while prev>0
            path=[path;RRTnode2(prev,1:2)];
            prev=RRTnode2(prev,3);
        end
        break;
    end
end

if size(Tree_State,1)<=0, error('no path found. maximum attempts reached'); end
pathOrigin=0;
for i=1:length(path)-1, pathOrigin=pathOrigin+pdist2(path(i,1:2),path(i+1,1:2)); end
t_elapse = toc;

plot (path(:,2),path(:,1),'MarkerEdgeColor','r','LineWidth', 2);
plot(path(:,2),path(:,1),'*r','LineWidth',6)
%plot optimal path
path_opt = optimization(map,path);
path_opt = [path_opt;q_goal];
plot(path_opt(:,2),path_opt(:,1),'-b','LineWidth',2)
%smooth path
if (~isempty(path_opt))
    P = SmoothPath(path_opt,map);
end

if (~isempty(path_opt))
    P1 = SmoothPath(P,map);
end
for i=2:length(P)
    plot([P(2,i);P(2,i-1)],[P(1,i);P(1,i-1)],'y-','LineWidth',2);
end

plot(P1(:,2),P1(:,1),'-r','LineWidth',2)

% B-spline curve
smooth = spcrv([[P(2,1) P(2,:) P(2,end)];[P(1,1) P(1,:) P(1,end)]],4); 
plot(smooth(1,:),smooth(2,:), '-g','LineWidth',2); 
%plot smooth path

l_opt = length(path_opt);
dis_opt = zeros(l_opt-1,1);
for i = 1: l_opt-1
    dis_opt(i) = pdist2(path_opt(i,:),path_opt(i+1,:));
end
dis_sum = sum(dis_opt);
P = P';
lp_opt = length(P);
disp_opt = zeros(lp_opt-1,1);
for i = 1: lp_opt-1
    disp_opt(i) = pdist2(P(i,:),P(i+1,:));
end
disp_sum = sum(disp_opt);

% smooth
%smooth = spcrv([[path_opt(1,2) path_opt(:,2)' path_opt(end,2)];[path_opt(1,1) path_opt(:,1)' path_opt(end,1)]],3); 
%plot(smooth(1,:),smooth(2,:), '-g','LineWidth',2); 
% Display info we need
fprintf('\nTime elapse=%f \n', t_elapse); 
fprintf('Path Original Length=%.3f \n',pathOrigin);
fprintf('Path Optimal Length 1st= %.3f \n',dis_sum);
fprintf('Path Optimal Length 2st= %.3f \n\n',disp_sum);
%%
function check=checkpoint(point,map)
if ~(point(1)>=1 && point(1)<=size(map,1) && point(2)>=1 && point(2)<=size(map,2) && map(point(1),point(2))==1)
    check=false;
else check=true;
end
end
%% 
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
%%
function [RRTree1,pathFound,extendFail]=rrtExtend(RRTree1,RRTree2,goal,delta,maxFailedAttempts,threshold,map)
pathFound=[]; %if path found, returns new node connecting the two trees, index of the nodes in the two trees connected
failedAttempts=0;
while failedAttempts<=maxFailedAttempts
    if rand < 0.2, 
        sample=rand(1,2) .* size(map); % random sample
    else
        sample=goal; % sample taken as goal to bias tree generation to goal
    end
    [Amin, Indx]=min( pdist2(RRTree1(:,1:2),sample) ,[],1); % find closest as per the function in the metric node to the sample
    closestNode = RRTree1(Indx,:);
    theta=atan2((sample(1)-closestNode(1)),(sample(2)-closestNode(2)));  % direction to extend sample to produce new node
    newPoint = double(int32(closestNode(1:2) + delta * [sin(theta)  cos(theta)]));
    if ~checkpath(closestNode(1:2), newPoint, map) % if extension of closest node in tree to the new point is feasible
        failedAttempts=failedAttempts+1;
        continue;
    end
    [Amin, Indx2]=min( pdist2(RRTree2(:,1:2),newPoint) ,[],1); % find closest in the second tree
    if pdist2(RRTree2(Indx2(1),1:2),newPoint)<threshold, % if both trees are connected
        pathFound=[newPoint Indx Indx2];extendFail=false;break; 
    end 
    [Amin, Indx3]=min( pdist2(RRTree1(:,1:2),newPoint) ,[],1); % check if new node is not already pre-existing in the tree
    if pdist2(newPoint,RRTree1(Indx3,1:2))<threshold, failedAttempts=failedAttempts+1;continue; end 
    RRTree1=[RRTree1;newPoint Indx(1)];extendFail=false;break; % add node
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

%% smoothpath (new)
function P = SmoothPath(path_opt,map)
P = path_opt';
[n,m] = size(P);
l = zeros(m,1);
for k = 2:m
    l(k) = norm(P(:,k)-P(:,k-1)) +l(k-1);
end
l_init = l(m);
iter =1;
itermax = 1000;
while iter <= itermax
    s1 = rand(1,1)*l(m);
    s2 = rand(1,1)*l(m);
    if s2< s1
        temps = s1;
        s1=s2;
        s2 = temps;
    end
    for k = 2:m
        if s1<l(k)
            i = k-1;
            break;
        end
    end
    for k=(i+1):m
        if s2<l(k)
            j = k-1;
            break;
        end
    end
    if (j<=i)
        iter =iter +1;
        continue;
    end
    t1 = (s1 -l(i))/(l(i+1)-l(i));
    gamma1 = (1-t1)*P(:,i)+t1*P(:,i+1);
    t2 = (s2 -l(j))/(l(j+1)-l(j));
    gamma2 = (1-t2)*P(:,j) + t2*P(:,j+1);
    check=checkpath(round(gamma1'),round(gamma2'),map);
    if (check ~= true)
        iter = iter +1;
        continue;
    end
    newP = [P(:,1:i) gamma1 gamma2 P(:,j+1:m)];
    clearvars P;
    P = newP;
    [n,m] = size(P);
    l = zeros(m,1);
    for k=2:m
        l(k) = norm(P(:,k)-P(:,k-1))+l(k-1);
    end
    iter = iter +1;
end
end