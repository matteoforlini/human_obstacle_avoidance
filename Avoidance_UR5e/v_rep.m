function v=v_rep(P,O,d_O,v_0)

dimension=size(P,1);
v=zeros(dimension,1);

for i=1:size(O,2)
    d=norm(P-O(:,i));
    nabla_d=(P-O(:,i))/d;
    if d<=d_O
        v=v+v_0*(1/d-1/d_O)/d^2*nabla_d;
    end
end
end
