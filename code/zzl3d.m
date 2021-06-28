%%draw 3D map
% x=-10:1:10;
% y=x';
% x=ones(size(y))*x;
% y=y*ones(size(y))';
% R=sqrt(x.^2+y.^2)+eps;
% z=sin(R)./R;
% mesh(z) 
%%%%%%%%%%%%
% x=[4229042.63      4230585.02    4231384.96    4231773.63    4233028.58    4233296.71    4235869.68    4236288.29];
% 
% y=[431695.4   441585.8      432745.6 436933.7      428734.4       431946.3 428705.0      432999.5];
% 
% z=[1.019 1.023      1.011      1.022      1.020      1.022      1.022      1.023];
% 
% scatter3(x,y,z)
%%%%%%%%%%%%%%%%%%%%
% ptCloud1 = pointCloud(rand(100,3,'single'));     
% ptCloud2 = pointCloud(1+rand(100,3,'single'));
%  
%   minDist = inf;
%   for i = 1 : ptCloud1.Count
%     point = ptCloud1.Location(i,:);
%     [~,dist] = findNearestNeighbors(ptCloud2,point,1);
%     if dist < minDist 
%         minDist = dist;
%     end
% end
%%%%%%%%%%%
% l1=500;l2=600;l3=400;l4=191.03;
% t1=linspace(-180,180,90)*pi/180;
% t2=linspace(-90,90,90)*pi/180;
% d3=linspace(-200,200,90);
% t4=linspace(-180,180,90)*pi/180;
% [T1,T2,D3]=ndgrid(t1,t2,d3);  % This will create matrices of 90x90x90 for each variable
% xM = round((-cos(T1).*cos(T2)).*((D3 + l2 + l3 + l4)));
% yM = round((-cos(T2).*sin(T1)).*(D3 + l2 + l3 + l4));
% zM = round((l1 - l4.*sin(T2) - sin(T2).*(D3 + l2 + l3)));
% plot3(xM(:),yM(:),zM(:),'.') % This is the plot type you should be using.
% % With a '.' as an argument to show only locations and not lines
% % Also, (:) converts any matrix into a list of its elements in one single column.
%%%%%%%%%%%%%%%%%
% x=linspace(0,1);
% 
% y=linspace(0,1);
% 
% [X,Y]=meshgrid(x,y);
% 
% for i=1:100
% 
% Z=linspace(i,i);
% 
% plot3(X,Y,Z);hold on
% 
% end
%%%%%%%%%%%%%%%%%%%
%map1 Ëæ»úµØ±í¡£
% clc
% clear all
% close all
% tic;
% h=[20,35,25,38,20,25,40];
% x0=[10,40,45,60,20,20,50];
% y0=[10,25,50,30,45,10,60];
% num = length(x0);
% xi=[5.5,8,5,4.5,5.5,3.5,5];
% yi=[5,7,6,5.5,6,4.5,5];
% Z2=CeatHill(num,h,x0,y0,xi,yi,80); 
% figure(1);
% surf(Z2); 
% shading flat;
% function [data]=CeatHill(N,h,x0,y0,xi,yi,num) 
% x=1:1:num;y=1:1:num;
% for m=1:num
%     for n=1:num
%         Sum=0;
%         for k=1:N
%            s=h(k)*exp(-((x(m)-x0(k))/xi(k))^2-((y(n)-y0(k))/yi(k))^2);
%            Sum=Sum+s;
%         end
%         data(m,n)=Sum;
%     end
% end
% 
% end

data.h =[ 5,35,25,38,30,10,50,30,15,45,34,40,20,20,25,30,-5,25];
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