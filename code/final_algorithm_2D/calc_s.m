function sp_s = calc_s(x,y)
dx = diff(x);
dy = diff(y);
ds = hypot(dx,dy);
sp_s = [0,cumsum(ds)];
end