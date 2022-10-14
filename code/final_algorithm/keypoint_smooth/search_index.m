function indx = search_index(x,t)
for i=1:length(x)-1
    if(t>=x(i)&& t<x(i+1))
        indx = i;
   end
end
if t==x(end)
   indx = length(x); 
end
end