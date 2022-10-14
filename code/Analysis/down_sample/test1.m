clear all;
%map=im2bw(imread('map7.png')); % input map read from a bmp file. for new maps write the file name here
%map=im2bw(imread('RRTsmart41.jpg'));
%map=im2bw(imread('maze.bmp'));
%map=im2bw(imread('map3.png'));

map=im2bw(imread('map4.bmp')); 
%map=im2bw(imread('map91.png'));
%map=im2bw(imread('map3.bmp')); 
% map=im2bw(imread('EBRRT5.jpg'));
%map = im2bw(imread('map1_c4_2.png'));
% map12
q_init=[10 490]; % source position in Y, X format
q_goal=[490 10]; % goal position in Y, X format

q_init=[10 40];
q_goal=[490 490];


stepsize=40; % size of each step of the RRT
disTh=20; % nodes closer than this threshold are taken as almost the same
maxFailedAttempts = 10000;

%% main program 
[time,path,path_down]= main(map,q_init,q_goal,stepsize,disTh,maxFailedAttempts);

%% plot figure

plot_figure(map,q_init,q_goal,path,path_down,[],[]);

path_ends = [];

for j=2:(length(path_down)-1)

    for i = 1:length(path)
        
       if path_down(j,:)==path(i,:)
           path_ends = [path_ends;path(i+1,:)];
           break;
       end    
    end
    
end
path_wrong =[];
for i = 1:length(path_ends)
path_wrong = [path_down(i,:);path_ends(i,:)] ;
    plot(path_wrong(:,2),path_wrong(:,1),'-.r','LineWidth',3);

end