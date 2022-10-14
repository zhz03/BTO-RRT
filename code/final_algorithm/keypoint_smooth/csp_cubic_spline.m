function [rx,ry,x,y]=csp_cubic_spline(waypoint)

x = waypoint(:,2)';
y = waypoint(:,1)';



sp_s = calc_s(x,y);
dt = 0.1;
s = [0:dt:sp_s(end)];
[ax,bx,cx,dx] = Spline_init(sp_s,x);
[ay,by,cy,dy] = Spline_init(sp_s,y);

rx = zeros(1,length(s));
ry = zeros(1,length(s));
for i=1:length(s)
    rx(i) = calc(s(i),sp_s,ax,bx,cx,dx);
    ry(i) = calc(s(i),sp_s,ay,by,cy,dy);
end

end

% function midp =midpoint(a,b)
% midp = (a+b)/2;
% end