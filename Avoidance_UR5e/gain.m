function     a_v=gain(d,r,r_min)
d1=r;
d2=(r_min+r)/2;
d3=r_min;

if d<=d2
    a_h=1;
else
    if d>=d1
        a_h=0;
    else
        a_h=0.5*(1+cos(pi*(d-d2)/(d1-d2)));
    end
end

if d>=d2
    a_v=0;
else
    a_v=((d-d2)/(d3-d2))^2;
end
    
end

