clear all;
map=im2bw(imread('RRTsmart41.jpg')); 
figure(3)
imshow(map);
%EBRRT1
q_init = [337 447];
q_goal = [328 850];
%EBRRT2
q_init = [54 52];
q_goal = [664 874];
%EBRRT3
q_init = [579 111];
q_goal = [344 868];
%EBRRT4
q_init = [591 55];
q_goal = [172 891];
%RRTsmart11
q_init = [530 297];
q_goal = [82 274];
%RRTsmart21
q_init = [600 423];
q_goal = [4 110];
%RRTsmart31
q_init = [31 202];
q_goal = [554 420];
%RRTsmart31
q_init = [85 346];
q_goal = [304 309];