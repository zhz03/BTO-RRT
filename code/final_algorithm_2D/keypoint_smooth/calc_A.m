function A = calc_A(h,ns)
A = zeros(ns,ns);
A(1,1) = 1.0;
for i=1:ns-1
   if i ~= (ns-1)
       A(i+1,i+1) = 2 * (h(i)+h(i+1));
   end
   A(i+1,i) = h(i);
   A(i,i+1) = h(i);
end

A(1,2) = 0;
A(ns,ns-1) = 0;
A(ns,ns) = 1;
end