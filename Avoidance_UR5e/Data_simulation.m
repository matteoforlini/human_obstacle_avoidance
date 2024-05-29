function [T,steps,n_frame,dt,v_1,v_2,Oi,Of,dOi,X_i,X_f]=Data_simulation(example)

switch example
    case 1
%% robot in movement
T=6;
steps=T*1000;
n_frame=10;
dt=T/steps;
v_1=0.1;
v_2=0.5;
Oi=[10000; 10000; 10000]; % simulated initial obstacle position (not used, obstacle are detected in realtime not simulated)
Of=[10000; 10000; 10000]; % final position simulated obstacle
dOi=[0; 0 ; 0];  %initial velocity
X_i=[-0.45 0.50 0.20 0 pi 0]';
X_f=[-0.45 -0.50 0.20 0 pi 0]';


    case 2
%% robot fixed
T=6;
steps=T*1000;
n_frame=10;
dt=T/steps;
v_1=0.1;
v_2=0.5;

Oi=[10000; 10000.; 10000]; 
Of=[10000; 10000.; 10000]; 
dOi=[0; 0 ; 0];  
X_i=[-0.45 0 0.2001 0 pi 0]';
X_f=[-0.45 0 0.20 0 pi 0]';
end

end