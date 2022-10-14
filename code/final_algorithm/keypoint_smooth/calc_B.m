function B = calc_B(h,ns,a)
B = zeros(1,ns);
for i=1:ns-2
    B(i+1) = 3.0 * (a(i+2)-a(i+1))/h(i+1) - 3.0 * (a(i+1)-a(i)) / h(i);
end
end