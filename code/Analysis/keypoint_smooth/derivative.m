clear
%% load data
load('path_up5.mat');
load('path_smooth5.mat');

%% visualize data
name = 'map1';
type = '.bmp';
fullname = [name,type];
map=im2bw(imread(fullname)); 

% figure(2)
% imshow(map); hold on;
% if (~isempty(path_up))
% %plot upsample path  
% plot(path_up(:,2),path_up(:,1),'*g','LineWidth',2);
% hold on 
% end

% if (~isempty(path_smooth))
% % plot smooth path
% plot(path_smooth(:,1),path_smooth(:,2), '-r','LineWidth',2);
% end

%% process data
smo_len = length(path_smooth);
up_len = length(path_up);

%linspace(1,10,20);
dist_up = zeros(up_len-1,1);

for i=1:up_len-1
    dist_up(i) = pdist2(path_up(i,:),path_up(i+1,:));
end
up_distsum = sum(dist_up);
step = up_distsum/smo_len;
path_up_den1 =[];
path_up_den2 =[];

for i=1:up_len-1
   insert1 = linspace(path_up(i,1),path_up(i+1,1),dist_up(i)*smo_len/up_distsum);
   path_up_den1= [path_up_den1,insert1(1:end-1)];
   
   insert2 = linspace(path_up(i,2),path_up(i+1,2),dist_up(i)*smo_len/up_distsum);
   path_up_den2= [path_up_den2,insert2(1:end-1)];
end

path_up_den = zeros(length(path_up_den1)+1,2);

path_up_den(1:end-1,1) = [path_up_den1'];
path_up_den(1:end-1,2) = [path_up_den2'];
path_up_den(end,1) = path_up(end,1);
path_up_den(end,2) = path_up(end,2);

%deriv_den = cal_deriveative(path_up(:,1),path_up(:,2));
deriv_den = cal_deriveative(path_up_den(:,1),path_up_den(:,2));
deriv_smo = cal_deriveative(path_smooth(:,2),path_smooth(:,1));

% index_smo = find(deriv_smo>1 );
% deriv_smo(index_smo)=0;
% index_smo = find(deriv_smo<-1);
% deriv_smo(index_smo)=0;

figure;
plot_deriv(deriv_den)
hold on
plot_deriv(deriv_smo)
legend('up sample optimization','key point smoother optimization');
xlabel('Sampling points on the trajectory');
%ytxt = ylabel('$\frac{dy}{dx}$');
ytxt = ylabel('$dy/dx$');
set(ytxt, 'Interpreter', 'latex');

len_smo = length(deriv_smo);
x_smo = 1:1:len_smo;
deriv_smo_2 = cal_deriveative1(x_smo',deriv_smo);
figure
plot_deriv(deriv_smo_2)
% plot_deriv1(deriv_smo)
% plot_deriv1(deriv_den)
function  yy = cal_deriveative(x,y)
diff_y = diff(y);
%diff_y (find(diff_y<0.1))=0;
diff_x = diff(x);
diff_x (find(diff_x<=0.5))=1;
yy = diff_y./diff_x;%导数值
end
function  yy = cal_deriveative1(x,y)
diff_y = diff(y);
%diff_y (find(diff_y<0.1))=0;
diff_x = diff(x);
yy = diff_y./diff_x;%导数值
end
function plot_deriv(yy)
plot(1:length(yy),yy,'LineWidth',2);
end

function plot_deriv1(yy)
figure;
plot(1:length(yy),yy,'LineWidth',2);
end
% figure(3)
% imshow(map); hold on;
% if (~isempty(path_up))
% %plot upsample path  
% plot(path_up_den(:,2),path_up_den(:,1),'-g','LineWidth',2);
% hold on 
% end
% 
% if (~isempty(path_smooth))
% % plot smooth path
% plot(path_smooth(:,1),path_smooth(:,2), '-r','LineWidth',2);
% end


% for i=1:up_len-1
%    insert1 = path_up(i,1):(step/2):path_up(i+1,1); 
%    path_up_den1 = [path_up_den1,insert1];
%    insert2 = path_up(i,1):(step/2):path_up(i+1,2); 
%    path_up_den2 = [path_up_den2,insert2];
% end