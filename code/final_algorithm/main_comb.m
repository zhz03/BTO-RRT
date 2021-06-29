function [time,path,path_down,path_up]=main_comb(map,q_init,q_goal,stepsize,disTh,maxFailedAttempts)


if ~(checkpoint(q_init,map) && checkpoint(q_goal,map)) 
    error('task point error! Reassign the task point'); 
end


RRTnode1=double([q_init -1]); % First RRT rooted at the source, representation node and parent index
RRTnode2=double([q_goal -1]); % Second RRT rooted at the goal, representation node and parent index

Expansion_state1=false; % sets to true if expansion after set number of attempts fails
Expansion_state2=false; % sets to true if expansion after set number of attempts fails

tic;

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

%% optimized
path_opt = optimization(map,path);
path_opt = [path_opt;q_goal];
%%
%smooth path
if (~isempty(path_opt))
    P = SmoothPath(path_opt,map);
end

%smooth = spcrv([[P(2,1) P(2,:) P(2,end)];[P(1,1) P(1,:) P(1,end)]],4);
t_lapse = toc;

P = P';
path_down = path_opt;
path_up = P;
%path_smooth = smooth;
time = t_lapse;
end