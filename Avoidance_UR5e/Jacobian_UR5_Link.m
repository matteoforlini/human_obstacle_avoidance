function J = Jacobian_UR5_Link(Q,index,x)
global tool

%index 1  2  3  4  5   6   7   
%link 1  2  3a  3b 4  5   6
q1 = (Q(1)); q2 = (Q(2)); q3 = (Q(3)); q4 = (Q(4)); q5 = (Q(5)); q6 = (Q(6));

% Real value
a_2=0.425;
a_3=0.39225;
d_1=0.1625; 
d_4=0.10915;
d_5=0.09465;
d_6=0.0996+tool; 
s_1=0.13585;
s_3=0.1197;

switch (index)
    case 1
        J=...
            [ -s_1*x*cos(q1), 0, 0, 0, 0, 0;...
            -s_1*x*sin(q1), 0, 0, 0, 0, 0;...
            0, 0, 0, 0, 0, 0;...
            0, 0, 0, 0, 0, 0;...
            0, 0, 0, 0, 0, 0;...
            1, 0, 0, 0, 0, 0];
    case 2
        J=...
            [ - s_1*cos(q1) - a_2*x*cos(q2)*sin(q1), -a_2*x*cos(q1)*sin(q2), 0, 0, 0, 0;...
            a_2*x*cos(q1)*cos(q2) - s_1*sin(q1), -a_2*x*sin(q1)*sin(q2), 0, 0, 0, 0;...
            0,         -a_2*x*cos(q2), 0, 0, 0, 0;...
            0,               -sin(q1), 0, 0, 0, 0;...
            0,                cos(q1), 0, 0, 0, 0;...
            1,                      0, 0, 0, 0, 0];
        
        
    case 3
        J=...
        [ s_3*x*cos(q1) - s_1*cos(q1) - a_2*cos(q2)*sin(q1), -a_2*cos(q1)*sin(q2),        0, 0, 0, 0;...
            s_3*x*sin(q1) - s_1*sin(q1) + a_2*cos(q1)*cos(q2), -a_2*sin(q1)*sin(q2),        0, 0, 0, 0;...
            0,         -a_2*cos(q2),        0, 0, 0, 0;...
            0,             -sin(q1), -sin(q1), 0, 0, 0;...
            0,              cos(q1),  cos(q1), 0, 0, 0;...
            1,                    0,        0, 0, 0, 0];
        
        
    case 4
        J=...
        [ s_3*cos(q1) - s_1*cos(q1) - a_2*cos(q2)*sin(q1) - a_3*x*cos(q2 + q3)*sin(q1), -cos(q1)*(a_2*sin(q2) + a_3*x*sin(q2 + q3)), -a_3*x*sin(q2 + q3)*cos(q1), 0, 0, 0;...
            s_3*sin(q1) - s_1*sin(q1) + a_2*cos(q1)*cos(q2) + a_3*x*cos(q2 + q3)*cos(q1), -sin(q1)*(a_2*sin(q2) + a_3*x*sin(q2 + q3)), -a_3*x*sin(q2 + q3)*sin(q1), 0, 0, 0;...
            0,          - a_2*cos(q2) - a_3*x*cos(q2 + q3),         -a_3*x*cos(q2 + q3), 0, 0, 0;...
            0,                                    -sin(q1),                    -sin(q1), 0, 0, 0;...
            0,                                     cos(q1),                     cos(q1), 0, 0, 0;...
            1,                                           0,                           0, 0, 0, 0];
        
        
    case 5
        J=...
        [ s_3*cos(q1) - s_1*cos(q1) - d_4*x*cos(q1) - a_3*cos(q2 + q3)*sin(q1) - a_2*cos(q2)*sin(q1), -cos(q1)*(a_3*sin(q2 + q3) + a_2*sin(q2)), -a_3*sin(q2 + q3)*cos(q1),        0, 0, 0;...
            s_3*sin(q1) - s_1*sin(q1) - d_4*x*sin(q1) + a_3*cos(q2 + q3)*cos(q1) + a_2*cos(q1)*cos(q2), -sin(q1)*(a_3*sin(q2 + q3) + a_2*sin(q2)), -a_3*sin(q2 + q3)*sin(q1),        0, 0, 0;...
            0,          - a_3*cos(q2 + q3) - a_2*cos(q2),         -a_3*cos(q2 + q3),        0, 0, 0;...
            0,                                  -sin(q1),                  -sin(q1), -sin(q1), 0, 0;...
            0,                                   cos(q1),                   cos(q1),  cos(q1), 0, 0;...
            1,                                         0,                         0,        0, 0, 0];
        
    case 6
        J=...
        [ s_3*cos(q1) - s_1*cos(q1) - d_4*cos(q1) - a_3*cos(q2 + q3)*sin(q1) - a_2*cos(q2)*sin(q1) + d_5*x*sin(q2 + q3 + q4)*sin(q1), -cos(q1)*(a_3*sin(q2 + q3) + a_2*sin(q2) + d_5*x*cos(q2 + q3 + q4)), -cos(q1)*(a_3*sin(q2 + q3) + d_5*x*cos(q2 + q3 + q4)), -d_5*x*cos(q2 + q3 + q4)*cos(q1),                          0, 0;...
            s_3*sin(q1) - s_1*sin(q1) - d_4*sin(q1) + a_3*cos(q2 + q3)*cos(q1) + a_2*cos(q1)*cos(q2) - d_5*x*sin(q2 + q3 + q4)*cos(q1), -sin(q1)*(a_3*sin(q2 + q3) + a_2*sin(q2) + d_5*x*cos(q2 + q3 + q4)), -sin(q1)*(a_3*sin(q2 + q3) + d_5*x*cos(q2 + q3 + q4)), -d_5*x*cos(q2 + q3 + q4)*sin(q1),                          0, 0;...
            0,            d_5*x*sin(q2 + q3 + q4) - a_2*cos(q2) - a_3*cos(q2 + q3),            d_5*x*sin(q2 + q3 + q4) - a_3*cos(q2 + q3),          d_5*x*sin(q2 + q3 + q4),                          0, 0;...
            0,                                                            -sin(q1),                                              -sin(q1),                         -sin(q1), -sin(q2 + q3 + q4)*cos(q1), 0;...
            0,                                                             cos(q1),                                               cos(q1),                          cos(q1), -sin(q2 + q3 + q4)*sin(q1), 0;...
            1,                                                                   0,                                                     0,                                0,         -cos(q2 + q3 + q4), 0];
        
        
    case 7
      J =...
[                                                                                s_3*cos(q1) - s_1*cos(q1) - d_4*cos(q1) - a_3*cos(q2 + q3)*sin(q1) - a_2*cos(q2)*sin(q1) + d_5*sin(q2 + q3 + q4)*sin(q1) - d_6*x*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)), -cos(q1)*(a_3*sin(q2 + q3) + a_2*sin(q2) + d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)), -cos(q1)*(a_3*sin(q2 + q3) + d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)), -cos(q1)*(d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)),                                                                                                                                                                                                                                                                                                                                                                                         d_6*x*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)),                                                   0;...
                                                                                 s_3*sin(q1) - s_1*sin(q1) - d_4*sin(q1) + a_3*cos(q2 + q3)*cos(q1) + a_2*cos(q1)*cos(q2) - d_5*sin(q2 + q3 + q4)*cos(q1) - d_6*x*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)), -sin(q1)*(a_3*sin(q2 + q3) + a_2*sin(q2) + d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)), -sin(q1)*(a_3*sin(q2 + q3) + d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)), -sin(q1)*(d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)),                                                                                                                                                                                                                                                                                                                                                                                     d_6*x*cos(q2 + q3 + q4)*cos(q5)*sin(q1) - d_6*x*cos(q1)*sin(q5),                                                   0;...
                                                                                                                                                                                                                                                                      0,            d_5*sin(q2 + q3 + q4) - a_2*cos(q2) - a_3*cos(q2 + q3) - d_6*x*cos(q2 + q3 + q4)*sin(q5),            d_5*sin(q2 + q3 + q4) - a_3*cos(q2 + q3) - d_6*x*cos(q2 + q3 + q4)*sin(q5),            d_5*sin(q2 + q3 + q4) - d_6*x*cos(q2 + q3 + q4)*sin(q5),                                                                                                                                                                                                                                                                                                                                                                                                                    -d_6*x*sin(q2 + q3 + q4)*cos(q5),                                                   0;...
                                                                                                                                                                                                                                                                      0,                                                                                            -sin(q1),                                                                              -sin(q1),                                                           -sin(q1),                                                                                                                                                                                                                                                                                                                                                                                                                          -sin(q2 + q3 + q4)*cos(q1), cos(q2 + q3 + q4)*cos(q1)*sin(q5) - cos(q5)*sin(q1);...
                                                                                                                                                                                                                                                                      0,                                                                                             cos(q1),                                                                               cos(q1),                                                            cos(q1),                                                                                                                                                                                                                                                                                                                                                                                                                          -sin(q2 + q3 + q4)*sin(q1), cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5);...
(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5))^2 + (sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) + sin(q2 + q3 + q4)*cos(q1)*cos(q6))^2 + (cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6))^2,                                                                                                   0,                                                                                     0,                                                                  0, (cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1))*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) - cos(q6)*(cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6))*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) - sin(q6)*(sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) + sin(q2 + q3 + q4)*cos(q1)*cos(q6))*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)),                          -sin(q2 + q3 + q4)*sin(q5)];
 
end
 %% Change reference system
 Rz180=[-1     0      0;
        0    -1      0;
        0     0      1];
 J(1:3,:)=Rz180*J(1:3,:);
end

