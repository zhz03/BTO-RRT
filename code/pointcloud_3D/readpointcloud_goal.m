%read point clouds
figure(1); 
ptCloud1 = pcread('hot-1pct.ply');
 pcshow(ptCloud1);
hold on;
xlabel('x');
ylabel('y');
zlabel('z');
% load goal points
% q_init = [175,130,120];
% q_goal = [70,215,100];
%hospital
q_init = [120,180,30];
q_goal = [20,40,30];

plot3(q_init(:,1),q_init(:,2),q_init(:,3),'*r');
plot3(q_goal(:,1),q_goal(:,2),q_goal(:,3),'*b');

drawsphere(q_init(:,1),q_init(:,2),q_init(:,3),1)
drawsphere(q_goal(:,1),q_goal(:,2),q_goal(:,3),1)
% K=10;
% [indices,dists] = findNearestNeighbors(ptCloud,q_goal,K);

% trisurf(tet, X(:,1), X(:,2), X(:,3))
%trisurf(loc(:,1),loc(:,2),loc(:,3));
%shading flat;
% x = double(loc(:,1));
% y = double(loc(:,2));
% z = double(loc(:,3));
% figure(3);
% scatter3(x,y,z);    %???
% [X,Y,Z]=griddata(x,y,z,linspace(min(x),max(x))',linspace(min(y),max(y))),'v4');
% figure(3);
% surf(X,Y,Z)

function drawsphere(a,b,c,R)
%% ????
% ?(a,b,c)????R???

    % ????
    [x,y,z] = sphere(20);

    % ????
    x = R*x; 
    y = R*y;
    z = R*z;

    % ????
    x = x+a;
    y = y+b;
    z = z+c;

    % ??mesh??
    %figure;
    axis equal;
    mesh(x,y,z);

    % ??surf??
    %figure;
%     axis equal;
%     surf(x,y,z);
end
