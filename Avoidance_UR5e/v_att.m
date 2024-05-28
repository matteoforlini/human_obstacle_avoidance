function v=v_att(P,G,d_G,v_0)

v_0=v_0/d_G;
d=norm(G-P);
if d<=d_G
    v=v_0*(G-P);
else
    v=v_0*d_G/d*(G-P);
end
end
