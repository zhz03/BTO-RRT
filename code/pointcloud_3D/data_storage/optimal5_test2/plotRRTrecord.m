clear all
% loading data
load('q_goal.mat');
load('q_init.mat');
load('RRTnode1.mat');
load('RRTnode2.mat');
load('path.mat');
load('path_opt.mat');
load('P.mat');
load('smooth.mat');

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

%% vedio recording
OUTPUT_TO_VIDEO = 1;
if OUTPUT_TO_VIDEO == 1
    v = VideoWriter('mapRRT_test2.avi');
    open(v)
end
time = 0;
for i=1:1:length(RRTnode1)
    plot3(RRTnode1(i,1),RRTnode1(i,2),RRTnode1(i,3),'*bl');
    plot3(RRTnode2(i,1),RRTnode2(i,2),RRTnode2(i,3),'*r');
    hold on;
    pause(0.05);
    time = time + 0.05;
    h_title = title(sprintf('run time: %4.2f',time));
    hold on;
        if OUTPUT_TO_VIDEO == 1
            im = frame2im(getframe(gcf));
            writeVideo(v,im);
            im = frame2im(getframe(gcf));
            writeVideo(v,im);
        end
end
%t = toc; % 3.35
%connect
%pause(0.1237);
plot3(path(:,1),path(:,2),path(:,3),'r','LineWidth',2);
for i=1:10
    
        if OUTPUT_TO_VIDEO == 1
            im = frame2im(getframe(gcf));
            writeVideo(v,im);
        end
        pause(0.01);
        time = time + 0.01;
        h_title = title(sprintf('run time: %4.2f',time));
        hold on;
end
%downsample

%pause(2.25-0.715);
plot3(path_opt(:,1),path_opt(:,2),path_opt(:,3),'-y','LineWidth',3);
frame = round((2.25-0.72)*30);
for i= 0:1:frame
        if OUTPUT_TO_VIDEO == 1
            im = frame2im(getframe(gcf));
            writeVideo(v,im);
        end
        time = time + (1/30);
        h_title = title(sprintf('run time: %4.2f',time));
        hold on;
end 
%upsample
%pause(2.25+0.715);

plot3(P(1,:),P(2,:),P(3,:),'-g','LineWidth',3);

frame = round((2.25+0.72)*30);
for i= 0:1:frame
        if OUTPUT_TO_VIDEO == 1
            im = frame2im(getframe(gcf));
            writeVideo(v,im);
        end
        time = time + (1/30);
        h_title = title(sprintf('run time: %4.2f',time));
        hold on;
end 
%smooth

%pause(0.5);
plot3(smooth(1,:),smooth(2,:),smooth(3,:), '-b','LineWidth',2);

frame = round((0.5)*30);  
for i=0:frame
        if OUTPUT_TO_VIDEO == 1
            im = frame2im(getframe(gcf));
            writeVideo(v,im);
        end
        time = time + (1/30);
        h_title = title(sprintf('run time: %4.2f',time));
        hold on;
end 
    
if OUTPUT_TO_VIDEO == 1
    close(v);
end