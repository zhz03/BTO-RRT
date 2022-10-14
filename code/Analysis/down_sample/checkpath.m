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