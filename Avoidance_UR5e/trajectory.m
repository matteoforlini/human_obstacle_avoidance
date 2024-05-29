%funciton for trajectory planning: Bezier of 3°, 4° and  5° 

function [t,X,dX,X_bez,dX_bez]=trajectory(order_Bezier_curve) 
    global X_i X_f dt T r r_min v0_rep v0_att k_e Q_i O

    x_i=X_i(1:3);
    x_f=X_f(1:3);
    d_O=r;
    d_G=r;
    level=10^-5;
    Oi=O(:,:,1);

    X_prel1(:,1)=x_i;
    X_prel2(:,1)=x_i;
    X_prel3(:,1)=x_i;
    X_prel4(:,1)=x_i;

    dX_prel1(:,1)=zeros(3,1);
    dX_prel2(:,1)=zeros(3,1);
    dX_prel3(:,1)=zeros(3,1);
    dX_prel4(:,1)=zeros(3,1);

    s_prel1(1)=0;
    s_prel2(1)=0;
    s_prel3(1)=0;
    s_prel4(1)=0;

    for m=1:1:4
        X_exit=x_i;
        while norm(X_exit-x_f)>level
            switch m    
                case 1
                    v_a=v_att(X_prel1(:,end),x_f,d_G,v0_att); %v_attraction
                    v_r=v_rep(X_prel1(:,end),Oi,d_O,v0_rep);  % v_repulsiv
                    v=v_a+v_r;                               
                    if v_r/norm(v_r)+v_a/norm(v_a)==0
                        vers_v=v/norm(v); 
                        vers_v_ort = null(vers_v(:).');        
                        v=v+(-1)^m*v0_rep*vers_v_ort(:,1); 
                    end 
                    X_new1=X_prel1(:,end)+dt*v;
                    X_prel1=[X_prel1 X_new1];
                    s_new1=s_prel1(:,end)+norm(dt*v);
                    s_prel1=[s_prel1 s_new1];
                    X_exit=X_new1;
                    dX_prel1=[dX_prel1 dt*v];

                case 2
                    v_a=v_att(X_prel2(:,end),x_f,d_G,v0_att); 
                    v_r=v_rep(X_prel2(:,end),Oi,d_O,v0_rep);  
                    v=v_a+v_r;                                
                    if v_r/norm(v_r)+v_a/norm(v_a)==0
                        vers_v=v/norm(v); 
                        vers_v_ort = null(vers_v(:).');       
                        v=v+(-1)^m*v0_rep*vers_v_ort(:,2); 
                    end
                    X_new2=X_prel2(:,end)+dt*v;
                    X_prel2=[X_prel2 X_new2];
                    s_new2=s_prel2(:,end)+norm(dt*v);
                    s_prel2=[s_prel2 s_new2];
                    X_exit=X_new2;
                    dX_prel2=[dX_prel2 dt*v];

                case 3
                    v_a=v_att(X_prel3(:,end),x_f,d_G,v0_att);
                    v_r=v_rep(X_prel3(:,end),Oi,d_O,v0_rep);
                    v=v_a+v_r;
                    if v_r/norm(v_r)+v_a/norm(v_a)==0
                        vers_v=v/norm(v);
                        vers_v_ort = null(vers_v(:).');        
                        v=v-(-1)^m*v0_rep*vers_v_ort(:,1);
                    end
                    X_new3=X_prel3(:,end)+dt*v;
                    X_prel3=[X_prel3 X_new3];
                    s_new3=s_prel3(:,end)+norm(dt*v);
                    s_prel3=[s_prel3 s_new3];
                    X_exit=X_new3;
                    dX_prel3=[dX_prel3 dt*v];

                case 4
                    v_a=v_att(X_prel4(:,end),x_f,d_G,v0_att);
                    v_r=v_rep(X_prel4(:,end),Oi,d_O,v0_rep);
                    v=v_a+v_r;
                    if v_r/norm(v_r)+v_a/norm(v_a)==0
                        vers_v=v/norm(v);
                        vers_v_ort = null(vers_v(:).');        
                        v=v-(-1)^m*v0_rep*vers_v_ort(:,2);
                    end
                    X_new4=X_prel4(:,end)+dt*v;
                    X_prel4=[X_prel4 X_new4];
                    s_new4=s_prel4(:,end)+norm(dt*v);
                    s_prel4=[s_prel4 s_new4];
                    X_exit=X_new4;
                    dX_prel4=[dX_prel4 dt*v];

            end
        end

    end

    choice = [s_prel1(end) s_prel2(end) s_prel3(end) s_prel4(end)];
    [m, ind] = min(choice);
    switch ind
        case 1
            X_prel=X_prel1;
            dX_prel=dX_prel1;
            s_prel=s_prel1;

        case 2
            
            X_prel=X_prel2;
            dX_prel=dX_prel2;
            s_prel=s_prel2;

        case 3
            X_prel=X_prel3;
            dX_prel=dX_prel3;
            s_prel=s_prel3;

        case 4
            X_prel=X_prel4;
            dX_prel=dX_prel4;
            s_prel=s_prel4;
    end

    s_prel=s_prel/s_prel(end);
    out = moto(0,0,0,1,0,0,0,1);
    t=out(:,1)';
    s=out(:,2)';
    ds=out(:,3)';
    X(1,:)=interp1(s_prel,X_prel(1,:),s);
    X(2,:)=interp1(s_prel,X_prel(2,:),s);
    X(3,:)=interp1(s_prel,X_prel(3,:),s);  

    dX(1,:)=interp1(s_prel,dX_prel(1,:),s);
    dX(2,:)=interp1(s_prel,dX_prel(2,:),s);
    dX(3,:)=interp1(s_prel,dX_prel(3,:),s); 

    %%%% bezer interpolation

    switch order_Bezier_curve
        case 3
            for i=1:size(s,2)
                S1(i,:)=[(1-s(i))^3  (s(i))^3];
                S2(i,:)=[3*s(i)*(1-s(i))^2  3*(s(i))^2*(1-s(i))];
                N(i,:)=[X(1,i) X(2,i) X(3,i)];
            end
            C1=[x_i';x_f'];
            C2=pinv(S2)*(N-S1*C1);
            coeff=[C1; C2]';
            X_A = coeff(:,1);
            X_B = coeff(:,2);
            X_A1 = coeff(:,3);
            X_B1 = coeff(:,4);
            V_A = 0;
            V_B = 0;
            X_bez = X_A.*((1-s).^3) +3*X_A1.*s.*((1-s).^2) +3*X_B1.*(s.^2).*(1-s) + X_B.*s.^3;
            dX_bez = (3*X_B.*s.^2).*ds - (3*X_B1.*s.^2).*ds - (3*X_A.*(s - 1).^2).*ds + (3*X_A1.*(s - 1).^2).*ds + (3*X_A1.*s.*(2*s - 2)).*ds - (6*X_B1.*s.*(s - 1)).*ds;

        case 4
            for i=1:size(s,2)
                S1(i,:)=[(1-s(i))^4  (s(i))^4 ];
                S2(i,:)=[4*s(i)*(1-s(i))^3  6*(s(i))^2*(1-s(i))^2 4*s(i)^3*(1-s(i))];
                N(i,:)=[X(1,i) X(2,i) X(3,i)];
            end
            C1=[x_i';x_f'];
            C2=pinv(S2)*(N-S1*C1);
            coeff=[C1; C2]';
            X_A = coeff(:,1);
            X_B = coeff(:,2);
            X_A1 = coeff(:,3);
            X_B1 = coeff(:,4);
            X_B2 = coeff(:,5);
            V_A = 0;
            V_B = 0;
            X_bez = X_A.*((1-s).^4) +4*X_A1.*s.*((1-s).^3) +6*X_B1.*(s.^2).*(1-s).^2 + 4*X_B2.*(s.^3).*(1-s) +X_B.*s.^4;
            dX_bez = ds.*(4*X_B.*s.^3 - 4*X_B2.*s.^3 + 4*X_A.*(s - 1).^3 - 4*X_A1.*(s - 1).^3 - 12*X_A1.*s.*(s - 1).^2 + 12*X_B1.*s.*(s - 1).^2 - 12*X_B2.*(s.^2).*(s - 1) + 6*X_B1.*(s.^2).*(2*s - 2));
        case 5
            for i=1:size(s,2)      
                S1(i,:)=[(1-s(i))^5  (s(i))^5 ];
                S2(i,:)=[5*s(i)*(1-s(i))^4  10*(s(i))^2*(1-s(i))^3 10*s(i)^3*(1-s(i))^2 5*s(i)^4*(1-s)];
                N(i,:)=[X(1,i) X(2,i) X(3,i)];
            end
            C1=[x_i';x_f'];
            C2=pinv(S2)*(N-S1*C1);
            coeff=[C1; C2]';
            X_A = coeff(:,1);
            X_B = coeff(:,2);
            X_A1 = coeff(:,3);
            X_B1 = coeff(:,4);
            X_B2 = coeff(:,5);
            X_C1 = coeff(:,6);
            V_A = 0;
            V_B = 0;
            X_bez = X_A.*((1-s).^5) +5*X_A1.*s.*((1-s).^4) +10*X_B1.*(s.^2).*(1-s).^3 + 10*X_B2.*(s.^3).*(1-s).^2 +5*X_C1*(s.^4).*(1-s)+X_B.*s.^5;
            dX_bez = ds.*(5*X_B.*s.^4 - 5*X_C1.*s.^4 - 5*X_A.*(s - 1).^4 + 5*X_A1.*(s - 1).^4 + 20*X_A1.*s.*(s - 1).^3 - 20*X_B1.*s.*(s - 1).^3 - 20*X_C1.*(s.^3).*(s - 1) + 10*X_B2.*(s.^3).*(2*s - 2) - 30*X_B1.*(s.^2).*(s - 1).^2 + 30*X_B2.*(s.^2).*(s - 1).^2);
    end
    eul_i=X_i(4:6);
    eul_f=X_f(4:6);
    E=eul_i+(eul_f-eul_i).*s;
    A=E(1,:);
    B=E(2,:);
    C=E(3,:);
    dA=diff(A)/dt;
    dA=[dA(1) dA];
    dB=diff(A)/dt;
    dB=[dB(1) dB];
    dC=diff(C)/dt;
    dC=[dC(1) dC];
    W =[dC.*cos(A).*sin(B) - dB.*sin(A);...
        dB.*cos(A) + dC.*sin(A).*sin(B);...
        dA + dC.*cos(B)];

    X=[X;E];
    X_bez=[X_bez;E];
    dX=[dX;W];
    dX_bez=[dX_bez;W];

end









   

