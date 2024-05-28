
function [O,dO]=obstacles_generation(Oi,Of,dOi,dt,T)

t=0:dt:T;

for i=1:size(t,2)
    O(:,:,i)=(Of-Oi)*t(i)/T+Oi;
    if i==1
        dO(:,:,i)=dOi;
    else
        dO(:,:,i)=(O(:,:,i)-O(:,:,i-1))/dt;
    end
end

