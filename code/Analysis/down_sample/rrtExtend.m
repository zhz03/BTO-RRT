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