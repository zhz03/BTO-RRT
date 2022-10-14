clear all
%% point cloud analysis
% read pointcloud map

%ptCloud1 = pcread('box-1pct.ply');
ptCloud1 = pcread('test-1pct.ply');
%ptCloud1 = pcread('hot-1pct.ply');
%ptCloud2 = pcread('box-10pct.ply');
%pcshow(ptCloud1);
%hold on;
xlabel('x');
ylabel('y');
zlabel('z');
tic;
data = ptCloud1.Location;
N = length(data);
radius = 0.5;
xr = ptCloud1.XLimits;
yr = ptCloud1.YLimits;
zr = ptCloud1.ZLimits;
rnp = randperm(N,round(0.01*N));
K= 3;
n = length(rnp);
distmin = zeros(n,1);
indx = zeros(n,1);
for i=1:n
    [A,dists] = findNearestNeighbors(ptCloud1,data(rnp(i),:),K);
    distmin(i) = max(dists);
    [indices,dists] = findNeighborsInRadius(ptCloud1,data(rnp(i),:),radius);
    indx(i) = length(indices);
end
figure(3);
histogram(distmin,round(n*0.1));
grid on;
ylabel('number of points');
xlabel('distance/ m');
title('Statistical Analysis of Point Cloud Map');

distmean = mean(distmin);
diststd = std(distmin);
saft_dist = distmean + 3 * diststd;
stepsize = 2 * saft_dist; 
% aveN = mean(indx);
% ptdensity1 = aveN/((4*pi*radius^3)/3);
% ptdensity2 = aveN/(pi*radius^2);
t_elapse = toc;
fprintf('\nTime elapse=%f \n\n',t_elapse);