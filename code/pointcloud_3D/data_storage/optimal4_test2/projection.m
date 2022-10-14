load('smooth.mat');
x = smooth(1,:);
y = smooth(2,:);
z = smooth(3,:);
t = 1:1:length(smooth);
n=zeros(size(t));
kxy = curvature(x,y);
[~,maxFlag] = max(kxy);
x_max = x(maxFlag);
y_max = y(maxFlag);


figure(5);
view(85,30); % for test2
hold on;
%plot3(smooth(1,:),smooth(2,:),smooth(3,:), '-b','LineWidth',2);
plot3(x,y,z,'.k');hold on;
plot3(n,y,z,'.r');hold on;
plot3(x,n,z,'.g');hold on;
plot3(x,y,n,'.b');
plot(x_max,y_max,'rp'); hold on;
hold off;
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

dfyx = diff(y)./diff(x);
dfyxabs = abs(dfyx);
gdyx = gradient(y,x);
midp = way_point(smooth);
xd = midp(1,:);
yd = midp(2,:);
zd = midp(3,:);
td = 1:1:length(midp);
nd=zeros(size(td));

figure(6)
plot3(xd,yd,zd,'kp');hold on;
plot3(nd,yd,zd,'.r');hold on;
plot3(xd,nd,zd,'.g');hold on;
plot3(xd,yd,nd,'.b');hold off;


function midp = way_point(path)
count = 0;
midp = path(:,1);
for i = 1:1:length(path)

    count = count +1;
    if(count == 10)
        count =0;
        midp = [midp,path(:,i)];
    end
end
midp = [midp,path(:,end)];
end

function k = curvature(x,y)

h1 = abs(diff([x])) ;
h = [h1 h1(end)];
ht = h;
yapp1 = gradient(y)./ht; %matlab????
yapp2 = del2(y)./ht; %matlab????
k = abs(yapp2)./(1+yapp1.^2).^(3/2);

end