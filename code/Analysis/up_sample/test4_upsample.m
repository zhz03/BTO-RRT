clear all
map=im2bw(imread('RRTsmart41.jpg'));
%map=im2bw(imread('map6.bmp'));
load('path_opt41.mat')
%load('path_down6.mat')
figure(1)
imshow(map); hold on;
%path_opt41 = [];
plot(path_opt41(:,2),path_opt41(:,1),"*-g",'LineWidth',2)
%plot(path_down6(:,2),path_down6(:,1),"*-g",'LineWidth',2)
hold on;
%% smoother
P = path_opt41';
%P = path_down6';
[n,m] = size(P);
l = zeros(m,1);
for k = 2:m
    l(k) = norm(P(:,k)-P(:,k-1)) +l(k-1);
end
l_init = l(m);
iter =1;
itermax = 100;
%%
while iter <= itermax
    s1 = rand(1,1)*l(m);
    s2 = rand(1,1)*l(m);
    if s2< s1
        temps = s1;
        s1=s2;
        s2 = temps;
    end
    for k = 2:m
        if s1<l(k)
            i = k-1;
            break;
        end
    end
    for k=(i+1):m
        if s2<l(k)
            j = k-1;
            break;
        end
    end
    if (j<=i)
        iter =iter +1;
        continue;
    end
    t1 = (s1 -l(i))/(l(i+1)-l(i));
    gamma1 = (1-t1)*P(:,i)+t1*P(:,i+1);
    t2 = (s2 -l(j))/(l(j+1)-l(j));
    gamma2 = (1-t2)*P(:,j) + t2*P(:,j+1);
    
    check=checkpath(round(gamma1'),round(gamma2'),map);
    if (check ~= true)
        iter = iter +1;
        continue;
    end
    newP = [P(:,1:i) gamma1 gamma2 P(:,j+1:m)];
    clearvars P;
    P = newP;
    [n,m] = size(P);
    l = zeros(m,1);
    for k=2:m
        l(k) = norm(P(:,k)-P(:,k-1))+l(k-1);
    end
    iter = iter +1;
end

plot(P(2,:),P(1,:),"*-r",'LineWidth',2)
disp_sum = cal_dist(P');
fprintf('Path length=%.3f \n',disp_sum);
title({['iteration times = ',num2str(itermax)],['length = ',num2str(disp_sum)]})
%% functions
function check=checkpoint(point,map)
if ~(point(1)>=1 && point(1)<=size(map,1) && point(2)>=1 && point(2)<=size(map,2) && map(point(1),point(2))==1)
    check=false;
else check=true;
end
end
%% 
function check=checkpath(q_near,q_new,map)
check=true;
theta=atan2(q_new(1)-q_near(1),q_new(2)-q_near(2));
for i=0:1:sqrt(sum((q_near-q_new).^2))
    poscheck=q_near+i.*[sin(theta) cos(theta)];
    if ~(checkpoint(ceil(poscheck),map) && checkpoint(floor(poscheck),map) && checkpoint([ceil(poscheck(1)) floor(poscheck(2))],map) && checkpoint([floor(poscheck(1)) ceil(poscheck(2))],map))
        check=false;
        break;
    end
    if ~checkpoint(q_new,map)
        check=false; 
    end
end
end