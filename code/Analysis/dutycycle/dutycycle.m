clear
N= 8;
Duty = zeros(1,N);
for k=1:N

namemap = 'map';
name = [namemap,num2str(k)];
type = '.bmp';
fullname = [name,type];
map=im2bw(imread(fullname)); 
bound = size(map);
count1 = 0;
count0 = 0;
for i =1:bound(1)
    
for j =1:bound(2)
    if (map(i,j)==0)
        count0 = count0 +1;
    else
        count1 = count1 + 1;
    end
    
end
end
dutycycle_map = count0/(count0+count1);
k
Duty(k) = dutycycle_map;
end