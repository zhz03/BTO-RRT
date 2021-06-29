clear 
map=im2bw(imread('map12.png'));
figure(2)
imshow(map);
[L,W] = size(map);
count = 0;
for i = 1: L
    for j = 1: W
        if (map(i,j) == 0)
            count = count + 1;
        end
    end
end
dutyc = count/(L*W)
