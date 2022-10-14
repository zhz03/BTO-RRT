
t = 0:0.1:10;
x = sin(t);

k = curvature(t,x);
figure(1)
plot(t,x,'.');
figure(2)
plot(k,'.r');


x0 = linspace(0.1,2,100);%x0,y0??????????????
y0 = 1./x0;
h1 = abs(diff([x0])) ;
h = [h1 h1(end)];
ht = h;
yapp1 = gradient(y0)./ht; %matlab????
yapp2 = del2(y0)./ht; %matlab????
k2 = abs(yapp2)./(1+yapp1.^2).^(3/2);
figure(3)
plot(k2)
title('The curvature of the curve')
[~,maxFlag] = max(k2);%??????
[~,minFlag] = min(k2);
x_min = x0(minFlag);
y_min = y0(minFlag);
x_max = x0(maxFlag);
y_max = y0(maxFlag);
%???? ???????
figure(4)
plot(x0,y0,'.-');
hold on;
plot(x_max,y_max,'rp')
plot(x_min,y_min,'rp')
title('max curvature')
xlabel('log10((norm(B*Xk-L)))')
ylabel('log10((norm(Xk)))')


function k = curvature(x,y)

h1 = abs(diff([x])) ;
h = [h1 h1(end)];
ht = h;
yapp1 = gradient(y)./ht; %matlab????
yapp2 = del2(y)./ht; %matlab????
k = abs(yapp2)./(1+yapp1.^2).^(3/2);

end
