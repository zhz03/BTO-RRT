function result = calc(t,s,a,b,c,d)
x = s;
if t<x(1)
    result = [];
elseif t>x(end)
    result = [];
end
i = search_index(x,t);
dx = t - x(i);
result = a(i) + b(i)*dx + c(i) * dx^2 + d(i) * dx^3;
%result = a(i) + c(i)*dx + b(i) * dx^2 + d(i) * dx^3;
end