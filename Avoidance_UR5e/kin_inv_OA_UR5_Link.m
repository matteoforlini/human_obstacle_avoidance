function [Q,r_vect,av_comp_time]=kin_inv_OA_UR5_Link(t,X,dX,O,dO,mode)
global  dt  r r_min v0_rep k_e Q_i k_v lambda_max eps dQ_max v_1 v_2 r_inf r_sup Socket_conn

tolleranza=0.90;
Q(:,1)=Q_i;
r_vect(1)=r_inf;
comp_time=0;
for i=1:size(t,2)-1
    tic
    o=O(:,:,i);
    V_o=dO(:,:,i);
    X_pl=X(:,i);
    dt=t(i+1)-t(i);
    J=Jacobian_UR5_Link(Q(:,i),7,1);
    S=svd(J); %singolar value decomposition
    s_min=S(6);
    if s_min<eps
        lambda=(1-(s_min/eps)^2)*lambda_max^2;
    else
        lambda=0;
    end
    pinvd_J=J'*inv(J*J'+lambda^2*eye(6)); %Damped Least-Squares
    J_par=J(1:3,:);
    S=svd(J_par);
    s_min=S(3);
    if s_min<eps
        lambda=(1-(s_min/eps)^2)*lambda_max^2;
    else
        lambda=0;
    end
    pinvd_J_par=J_par'*inv(J_par*J_par'+lambda^2*eye(3)); %Damped Least- Squares
    [X_ac,robot_point]=kin_dir_UR5_Link(Q(:,i));
    robot_point=robot_point(1:3,:);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    robot_point_g=[];
    robot_point_g(:,1)=robot_point(:,2);%A
    robot_point_g(:,2)=robot_point(:,3);%B
    robot_point_g(:,3)=robot_point(:,6);%C
    robot_point_g(:,4)=robot_point(:,7);%C1
    robot_point_g(:,5)=robot_point(:,10);%D
    robot_point_g(:,6)=robot_point(:,11);%E
    robot_point_g(:,7)=robot_point(:,12);%F
    robot_point_g(:,8)=robot_point(:,13);%G==EE
    [k,dist,x]=dis_link(robot_point_g,o);
    [k_ee,d_ee] = dsearchn(o',robot_point(:,13)');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
 
    P_o_ee=o(:,k_ee);  
    v_o_ee=V_o(:,k_ee);    
    P_r_ee=robot_point(:,13);
        
    [d_min,index]=min(dist);
    P_r=robot_point_g(:,index)+x(index)*(robot_point_g(:,index+1)-robot_point_g(:,index));
    %P_r=robot_point(:,index+3);
    P_o=o(:,k(index));
    v_o=V_o(:,k(index));
    %%%%%%%%%%%%%%%%%%
    if norm(v_o)<v_1
        r_min=r_inf;
    elseif norm(v_o)>v_2
        r_min=r_sup;
    else
        r_min=r_inf+(r_sup-r_inf)/(v_2-v_1)*(norm(v_o)-v_1);
    end
    r=4/3*r_min;
    %%%%%%%%%%%%%%%
    if  or (d_ee<tolleranza*r_min,d_min<tolleranza*r_min)
        disp 'fail'
    end
    a_v=gain(d_min,r,r_min);
    J0=Jacobian_UR5_Link(Q(:,i),index,x(index));   
    vector=(P_r-P_o-k_v*v_o*dt);   
    d0=vector/norm(vector);
    %plot3(P_r(1,1),P_r(2,1),P_r(3,1),'ob');
    N=eye(6)-pinvd_J*J;
    J0_par=J0(1:3,:);
    S=svd(J0_par);
    s_min=S(3);
    if s_min<eps
        lambda=(1-(s_min/eps)^2)*lambda_max^2;
    else
        lambda=0;
    end
    pinvd_J0_par=J0_par'*inv(J0_par*J0_par'+lambda^2*eye(3)); %Damped Least- Squares

    e(1:3,1)=X_pl(1:3)-X_ac(1:3);
    eul_pl=X_pl(4:6);
    eul_ac=X_ac(4:6);
    R_pl=eul2rotm(eul_pl','ZYZ');
    R_ac=eul2rotm(eul_ac','ZYZ');
    e(4:6,1)=(0.5*(cross(R_ac(:,1),R_pl(:,1))+...
    cross(R_ac(:,2),R_pl(:,2))+...
    cross(R_ac(:,3),R_pl(:,3))))/(2*pi);
    vector=(P_r_ee-P_o_ee-k_v*v_o_ee*dt);
    d_rep=vector/norm(vector);
    

    switch mode
        case 3 %3 dof free
            J3=[J0_par;J(4:6,:)];
            S=svd(J3); %singolar value decomposition.
            s_min=S(6);
            if s_min<eps
                lambda=(1-(s_min/eps)^2)*lambda_max^2;
            else
                lambda=0;
            end
            pinvd_J3=J3'*inv(J3*J3'+lambda^2*eye(6)); %Damped Least-Squares
            
            dQ=(pinvd_J*(dX(:,i)+k_e*e)+...
                +pinvd_J3*(a_v*v0_rep*[d0; zeros(3,1)]));
        case 2
            J4=[J0_par;J(4:5,:)];
        S=svd(J4); %singolar value decomposition. 
            s_min=S(5);
            if s_min<eps
                lambda=(1-(s_min/eps)^2)*lambda_max^2;
            else
                lambda=0;
            end
            pinvd_J4=J4'*inv(J4*J4'+lambda^2*eye(5)); %Damped Least-Squares
            dQ=(pinvd_J*(dX(:,i)+k_e*e)+...
                +pinvd_J4*(a_v*v0_rep*[d0; zeros(2,1)]));
            
        otherwise %6 dof free
        dQ=(pinvd_J*(dX(:,i)+k_e*e)+...
        +pinvd_J0_par*(a_v*v0_rep*d0));
    end

for j=1:6
    if abs(dQ(j))>dQ_max
        dQ(j)=dQ_max*sign(dQ(j));
    end
end
    Q(:,i+1)=Q(:,i)+dt*dQ;
    r_vect(i+1)=r_min;
    comp_time=toc+comp_time;

end
av_comp_time=comp_time/i;
end
    