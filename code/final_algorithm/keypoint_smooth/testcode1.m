test = [0,0;0,5;-8,12;-8,27];
test = [0,0;3,5;-8,12;-8,27];
[rx,ry,x,y]=csp_cubic_spline(test);

figure(5)
plot(x(1,:),y(1,:),'*r','LineWidth',4);
hold on 
plot(x(1,:),y(1,:),'-g','LineWidth',2);
hold on 
plot(rx(1,:),ry(1,:),'-b','LineWidth',2);
hold on
axis equal