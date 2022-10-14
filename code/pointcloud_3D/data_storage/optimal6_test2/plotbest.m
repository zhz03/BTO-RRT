figure(2);
load('smooth_test2.mat');
smooth1 = smooth;
load('smooth.mat');
ptCloud = pcread('box-1pct.ply');
pcshow(ptCloud);
hold on;
view(85,30); % for test2
xlabel('x');
ylabel('y');
zlabel('z');
plot3(smooth1(1,:),smooth1(2,:),smooth1(3,:), '-r','LineWidth',2);
plot3(smooth(1,:),smooth(2,:),smooth(3,:), '-b','LineWidth',2);


load('q_goal.mat');
load('q_init.mat');
load('RRTnode1.mat');
load('RRTnode2.mat');
load('path.mat');
load('path_opt.mat');
load('P.mat');
load('smooth.mat');
load('presmooth_test2.mat');

figure(1);
ptCloud = pcread('box-1pct.ply');
pcshow(ptCloud);
hold on;
view(85,30); % for test2
set(0,'defaultfigurecolor','w')

plot3(q_init(:,1),q_init(:,2),q_init(:,3),'*b','LineWidth',6);
plot3(q_goal(:,1),q_goal(:,2),q_goal(:,3),'*r','LineWidth',6);

xlabel('x');
ylabel('y');
zlabel('z');

tic;
for i=1:1:length(RRTnode1)
    plot3(RRTnode1(i,1),RRTnode1(i,2),RRTnode1(i,3),'*bl');
    plot3(RRTnode2(i,1),RRTnode2(i,2),RRTnode2(i,3),'*r');
    hold on;
    pause(0.05);
end
t = toc % 3.35
%connect
pause(0.1237);
plot3(path(:,1),path(:,2),path(:,3),'r','LineWidth',2);
%downsample
pause(2.25-0.715);
plot3(path_opt(:,1),path_opt(:,2),path_opt(:,3),'-y','LineWidth',3);
%upsample
pause(2.25+0.715);
plot3(P(1,:),P(2,:),P(3,:),'-g','LineWidth',3);
%smooth
pause(0.5);
%plot3(smooth(1,:),smooth(2,:),smooth(3,:), '-b','LineWidth',2);
plot3(smooth1(1,:),smooth1(2,:),smooth1(3,:), '-b','LineWidth',2);