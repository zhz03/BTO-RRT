map=im2bw(imread('test2d1.png')); 
q_init=[50 50];
q_goal=[306 1526];


figure(2)
imshow(map); hold on;
plot(q_init(2),q_init(1),'*b','LineWidth',4);
plot(q_goal(2),q_goal(1),'*r', 'LineWidth', 4);
