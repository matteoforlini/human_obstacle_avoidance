function [X, points]=kin_dir_UR5_Link(Q)
global tool


a_2=0.425;
a_3=0.39225;
d_1=0.1625; 
d_4=0.10915;
d_5=0.09465;
d_6=0.0996+tool; 
s_1=0.13585;
s_3=0.1197;


q_1=Q(1); q_2=Q(2);q_3=Q(3);q_4=Q(4);q_5=Q(5);q_6=Q(6);


%point A
R_1=[cos(q_1)       -sin(q_1)         0;
     sin(q_1)        cos(q_1)         0;
     0                 0              1];
 
 %point B
%  R_2=[cos(q_2)     0        sin(q_2);
%       0            1              0 ;
%      -sin(q_2)     0        cos(q_2)];

 R_2=[-sin(q_2)     0        cos(q_2);
      0            1              0 ;
     -cos(q_2)     0        -sin(q_2)];

%point C
R_3=[cos(q_3)     0        sin(q_3);
     0            1            0   ;
     -sin(q_3)    0        cos(q_3)];
 
 %point B1
 R_B1=[1          0           0;
       0          1           0;
       0          0           1];

 %point D
R_4=[-sin(q_4)     0          +cos(q_4);
    0             1             0    ;
    -cos(q_4)      0         -sin(q_4)];

 %point E
 R_5=[cos(q_5)    -sin(q_5)     0;
     sin(q_5)      cos(q_5)     0;
       0              0         1]; 
 
 %point F
R_6=[cos(q_6)      0        sin(q_6);
    0              1              0 ;
    -sin(q_6)      0        cos(q_6)];

%matrice di rotazione dell'end effector

 R_7=[  0   -1     0;
        0    0     1;
       -1    0    0];

% R_7=[1     0      0;
%      0     0      1;
%      0    -1     0];

 %vector joint translation
p_1=[0;0;d_1];                                             %point A
p_2=[0;s_1;0];                                              %point B
p_3=[0;0;a_2/3];                                           %point C
p_4=[0;0;a_3/3];                                           %point D
p_5=[0;d_4;0];                                             %point E  
p_6=[0;0;d_5];                                             %point F
p_7=[0;d_6;0];                                             %point G 


T_1 = [R_1,p_1;zeros(1,3),1];
T_2 = [R_2,p_2;zeros(1,3),1];
T_3 = [R_3,p_3;zeros(1,3),1];
T_4 = [R_4,p_4;zeros(1,3),1];
T_5 = [R_5,p_5;zeros(1,3),1];
T_6 = [R_6,p_6;zeros(1,3),1];

%base point
%O =[0 0 0 pi 0 0]; 
O =[0 0 0 0 0 0]; 
%point A (joint 1)
A(1:3)=p_1;
A(4:6)=ang_eulero(R_1);

%point B (joint 2)
T_02=T_1*T_2;
B(1:3)=T_02(1:3,4);
B(4:6)=ang_eulero(T_02(1:3,1:3));

%point B1 (sul link 2)
p_B1=[0;0;a_2/3];
T_B1=[R_B1,p_B1;zeros(1,3),1];
T_0B1=T_02*T_B1;
B1(1:3)=T_0B1(1:3,4);
B1(4:6)=ang_eulero(T_0B1(1:3,1:3));

%point B2 (sul link 2)
p_B2=[0;0;a_2/3];
T_B2=[R_B1,p_B2;zeros(1,3),1];
T_0B2=T_0B1*T_B2;
B2(1:3)=T_0B2(1:3,4);
B2(4:6)=ang_eulero(T_0B1(1:3,1:3));


%point C (joint 3)
T_03=T_0B2*T_3;
C(1:3)=T_03(1:3,4);
C(4:6)=ang_eulero(T_03(1:3,1:3));

%point C1 (intermediate tra C e D)
p_c1=[0;-s_3;0];
R_c1=R_B1;
T_c1=[R_c1,p_c1;zeros(1,3),1];
T_0c1=T_03*T_c1;
C1(1:3)=T_0c1(1:3,4);
C1(4:6)=ang_eulero(T_0c1(1:3,1:3));


%point C2 (intermediate  C  D)
p_C2=[0;0;a_3/3]; 
T_C2=[R_B1,p_C2;zeros(1,3),1];
T_0C2=T_0c1*T_C2;
C2(1:3)=T_0C2(1:3,4);
C2(4:6)=ang_eulero(T_0C2(1:3,1:3));

%point C3 (intermediate  C  D)
p_C3=[0;0;a_3/3]; 
T_C3=[R_B1,p_C3;zeros(1,3),1];
T_0C3=T_0C2*T_C3;
C3(1:3)=T_0C3(1:3,4);
C3(4:6)=ang_eulero(T_0C3(1:3,1:3));

%point D (joint 4)
T_04=T_0C3*T_4;
D(1:3)=T_04(1:3,4);
D(4:6)=ang_eulero(T_04(1:3,1:3));

%point E (joint 5) 
T_05=T_04*T_5;
E(1:3)=T_05(1:3,4);
E(4:6)=ang_eulero(T_05(1:3,1:3));

%point F (ginto 6) 
 T_06=T_05*T_6;
 F(1:3)=T_06(1:3,4);
 F(4:6)=ang_eulero(T_06(1:3,1:3));
 
 %point G (end effector)
%  R7=R_B1;
 T_7=[R_7,p_7;zeros(1,3),1];
 T_07=T_06*T_7;
 G(1:3)=T_07(1:3,4);
 G(4:6)=ang_eulero(T_07(1:3,1:3));
 
 
 
 X=G';
 points=[O' A' B' B1' B2' C' C1' C2' C3' D' E' F' G'];


 Rz180=[-1     0      0;
        0    -1      0;
        0     0      1];
 X(1:3)=Rz180*X(1:3);
 points(1:3,:)=Rz180*points(1:3,:);
end


 