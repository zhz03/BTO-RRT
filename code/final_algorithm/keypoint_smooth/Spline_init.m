function [a,b,c,d] = Spline_init(s,xy)
x = s;
y = xy;
ns = length(x);
h = diff(x);
a = y;

A = calc_A(h,ns);
B = calc_B(h,ns,a);
c = A\B';

b = zeros(ns-1,1);
d = zeros(ns-1,1);
for i=1:(ns-1)
    b(i) = (a(i+1)-a(i))/h(i) - h(i) * (c(i+1)+2.0*c(i)) /3.0;
    d(i) = (c(i+1) -c(i))/(3*h(i));
end
    
end