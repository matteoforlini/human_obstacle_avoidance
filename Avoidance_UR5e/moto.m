function [Output_moto] = moto(qi,vi,ai,qf,vf,af,ti,choice)
global dt T 

tf=ti+T;
samples=round(T/dt);
t = (ti:T/samples:tf);

if choice == 1 
a0 = qi;
a1 = vi;
a2 = ai/2;
a3 = (20*(qf-qi)-(8*vf+12*vi)*T-(3*af-ai)*T^2)/(2*T^3);
a4 = (30*(qi-qf)+(14*vf+16*vi)*T+(3*af-2*ai)*T^2)/(2*T^4);
a5 = (12*(qf-qi)-6*(vf+vi)*T-(af-ai)*T^2)/(2*T^5);

for k = 1:size(t,2)
  M(k,:) = a0 + a1*(t(k)-ti) + a2*(t(k)-ti)^2 + a3*(t(k)-ti)^3 + a4*(t(k)-ti)^4 + a5*(t(k)-ti)^5;
  V(k,:) = a1 + 2*a2*(t(k)-ti) + 3*a3*(t(k)-ti)^2 + 4*a4*(t(k)-ti)^3 + 5*a5*(t(k)-ti)^4;
  A(k,:) = 2*a2 + 6*a3*(t(k)-ti) + 12*a4*(t(k)-ti)^2 + 20*a5*(t(k)-ti)^3;
end

elseif choice == 2  %trapezoidal speed
    Ta = T/4;
    vcost = (qf-qi)/(T-Ta);
    for k = 1:size(t,2)
        if t(k)<=(ti+Ta)
           M(k,:) = qi + vcost*(t(k)-ti)^2/(2*Ta);
           V(k,:) = vi + vcost*(t(k)-ti)/Ta;
           A(k,:) = vcost/Ta;
        elseif t(k)>(ti+Ta) && t(k)<=(tf-Ta)
           M(k,:) = qi + vcost*(t(k)-ti-Ta/2);
           V(k,:) = vi + vcost;
           A(k,:) = zeros(1,7);
        elseif t(k)>(tf-Ta)
           M(k,:) = qf - vcost*(tf-t(k))^2/(2*Ta);
           V(k,:) = vf + vcost*(tf-t(k))/Ta;
           A(k,:) = -vcost/Ta;
        end
    end
       
end

Output_moto = [t',M,V,A];

end



