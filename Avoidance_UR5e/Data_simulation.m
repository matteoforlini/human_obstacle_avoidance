function [T,steps,n_frame,dt,v_1,v_2,Oi,Of,dOi,X_i,X_f]=Data_simulation(example)

T = 6;

switch example
    case 0
%% Rettilineo
% T=6;
steps=T*1000;
n_frame=10;
dt=T/steps;
v_1=0.1;
v_2=0.5;

Oi=[10000; 10000.; 10000]; % [x1 x2; y1 y2; z1 z2] %x1 first obstacle
Of=[10000; 10000; 10000]; % [x1 x2; y1 y2; z1 z2]
dOi=[0; 0 ; 0];  %condizione iniziale velocità
X_i=[-0.45 0.50 0.30 0 pi 0]';
X_f=[-0.45 -0.50 0.30 0 pi 0]';

    case 1
%% Ostacolo fisso
% T=6;
steps=T*1000;
n_frame=10;
dt=T/steps;
v_1=0.1;
v_2=0.5;

Oi=[-0.50 -0.50; -0.15 0.15; 0.40 0.40]; % [x1 x2; y1 y2; z1 z2] %x1 first obstacle
Of=[-0.50 -0.50; -0.151 0.15; 0.40 0.40]; % [x1 x2; y1 y2; z1 z2]
dOi=[0 0; 0 0; 0 0];  %condizione iniziale velocità
X_i=[-0.45 0.50 0.20 0 pi 0]';
X_f=[-0.45 -0.50 0.20 0 pi 0]';
    case 2
%% Ostacolo movimento
% T=6;
steps=T*1000;
n_frame=10;
dt=T/steps;
v_1=0.1;
v_2=0.5;

Oi=[0; -0.56; 0.23]; % [x1 x2; y1 y2; z1 z2] %x1 first obstacle
Of=[0.001; -0.56; 0.23]; % [x1 x2; y1 y2; z1 z2]
dOi=[0; 0 ; 0];  %condizione iniziale velocità
X_i=[-0.45 0 0.201 0 pi 0]';
X_f=[-0.45 0 0.20 0 pi 0]';
end

end