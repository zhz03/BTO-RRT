clear 
Mean_Dist_RRT = zeros(8,1);
Std_Dist_RRT = zeros(8,1);

Mean_Time_RRT = zeros(8,1);
Std_Time_RRT = zeros(8,1);

for i=1:8
    %i = 1;
    namemap = 'map';
    name = [namemap,num2str(i)];
    foldername = 'RRT_star\';
    mattype = '.mat';
    Distname = '_Dist_RRT';
    Tname = 'Time';
    fullname1 = [[[foldername,name],Distname],mattype];
    fullname2 = [[[foldername,name],Tname],mattype];
    load(fullname1);
    load(fullname2);
    Mean_Dist_RRT(i) = mean(Dist_RRT);
    Std_Dist_RRT(i) = std(Dist_RRT);
    Mean_Time_RRT(i) = mean(Time_RRT);
    Std_Time_RRT(i) = std(Time_RRT);    
    
end

figure;
plorerrorbar(Mean_Dist_RRT,Std_Dist_RRT,500,1300);