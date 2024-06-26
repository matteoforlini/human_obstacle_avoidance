function [Ctot, Cam]=data_unification(c1,c2,c3, Ctot_old)

if Ctot_old==1
    Ctot_old=zeros(4,33); %to set the first iteration
end

id_camera1=c1(:,1); %remove the first element that is camera id
c1=c1(:,2:end); 
i=size(c1,2)/5;

id_row1=c1(:,1:i);
vis_row1=c1(:,(1*i+1):(1*i+i));
c1x_row=c1(:,(2*i+1):(2*i+i));
c1y_row=c1(:,(3*i+1):(3*i+i));
c1z_row=c1(:,(4*i+1):(4*i+i));

Ctot1=[]; 
Ctot1(1,:)=id_row1;
Ctot1(2,:)=vis_row1;
Ctot1(3,:)=c1x_row;
Ctot1(4,:)=c1y_row;
Ctot1(5,:)=c1z_row;


id_camera2=c2(:,1);

c2=c2(:,2:end); %remove the first element that is camera id
i=size(c2,2)/5;

id_row2=c2(:,1:i);
vis_row2=c2(:,(1*i+1):(1*i+i));
c2x_row=c2(:,(2*i+1):(2*i+i));
c2y_row=c2(:,(3*i+1):(3*i+i));
c2z_row=c2(:,(4*i+1):(4*i+i));

Ctot2=[];
Ctot2(1,:)=id_row2;
Ctot2(2,:)=vis_row2;
Ctot2(3,:)=c2x_row;
Ctot2(4,:)=c2y_row;
Ctot2(5,:)=c2z_row;

if c3==[1.0] %c3 is equal to 1 when the third camera is not working, used to adapt easily the program to use two cameras instead of three
    Ctot3=[]; 
    Ctot3(1,1:33)=(1:1:33)
    Ctot3(2:5,1:33)=nan(4,33)
else
    id_camera3=c3(:,1);

    c3=c3(:,2:end); %levo da c1 il primo elemento che è l'id
    i=size(c3,2)/5;

    id_row3=c3(:,1:i);
    vis_row3=c3(:,(1*i+1):(1*i+i));
    c3x_row=c3(:,(2*i+1):(2*i+i));
    c3y_row=c3(:,(3*i+1):(3*i+i));
    c3z_row=c3(:,(4*i+1):(4*i+i));

    Ctot3=[];
    Ctot3(1,:)=id_row3;
    Ctot3(2,:)=vis_row3;
    Ctot3(3,:)=c3x_row;
    Ctot3(4,:)=c3y_row;
    Ctot3(5,:)=c3z_row;
end
% Fill the 5x33 matrices of Cam1...Cam3 with NaN values if the corresponding body ID is not detected
% This ensures that all matrices maintain a consistent size of 5x33
if size(Ctot1,2)~=33
    Ctot1=[Ctot1,zeros(5,(33-size(Ctot1,2)))];
end
if Ctot1(1,1)==0
    for i=1:32
        if Ctot1(1,i)~=(Ctot1(1,i+1)-1)
            Ctot1=[Ctot1(:,1:i),[i;nan;nan;nan;nan],Ctot1(:,i+1:end)];
        end
    end
    Ctot1=Ctot1(:,1:33);
else
    Ctot1=[[0;nan;nan;nan;nan],Ctot1];
    for i=1:32
        if Ctot1(1,i)~=(Ctot1(1,i+1)-1)
            Ctot1=[Ctot1(:,1:i),[i;nan;nan;nan;nan],Ctot1(:,i+1:end)];
        end
    end
    Ctot1=Ctot1(:,1:33);
end

if size(Ctot2,2)~=33
    Ctot2=[Ctot2,zeros(5,(33-size(Ctot2,2)))];
end
if Ctot2(1,1)==0
    for i=1:32
        if Ctot2(1,i)~=(Ctot2(1,i+1)-1)
            Ctot2=[Ctot2(:,1:i),[i;nan;nan;nan;nan],Ctot2(:,i+1:end)];
        end
    end
    Ctot2=Ctot2(:,1:33);
else
    Ctot2=[[0;nan;nan;nan;nan],Ctot2];
    for i=1:32
        if Ctot2(1,i)~=(Ctot2(1,i+1)-1)
            Ctot2=[Ctot2(:,1:i),[i;nan;nan;nan;nan],Ctot2(:,i+1:end)];
        end
    end
    Ctot2=Ctot2(:,1:33);
end

if size(Ctot3,2)~=33
    Ctot3=[Ctot3,zeros(5,(33-size(Ctot3,2)))];
end
if Ctot3(1,1)==0
    for i=1:32
        if Ctot3(1,i)~=(Ctot3(1,i+1)-1)
            Ctot3=[Ctot3(:,1:i),[i;nan;nan;nan;nan],Ctot3(:,i+1:end)];
        end
    end
    Ctot3=Ctot3(:,1:33);
else
    Ctot3=[[0;nan;nan;nan;nan],Ctot3];
    for i=1:32
        if Ctot3(1,i)~=(Ctot3(1,i+1)-1)
            Ctot3=[Ctot3(:,1:i),[i;nan;nan;nan;nan],Ctot3(:,i+1:end)];
        end
    end
    Ctot3=Ctot3(:,1:33);
end

%fusion
Ctot=[];
for i=1:33
    vis_vector=[Ctot1(2,i),Ctot2(2,i),Ctot3(2,i)]; % Arrange three numbers in a row: the visibility indices of the three cameras for each joint
    [val, idx] = max(vis_vector);
    
    if (i==17 || i==16) && isnan(val) %id 16 and 17 are of the two wrists
        Ctot=[Ctot,Ctot_old(:,i)]; 
    else
        if  idx==1
            Ctot=[Ctot,Ctot1(2:end,i)];
        elseif idx==2
            Ctot=[Ctot,Ctot2(2:end,i)];
        elseif idx==3
            Ctot=[Ctot,Ctot3(2:end,i)];
        end
    end


    if i==17 %This if statement determines which camera has the best visibility index of the right wrist
        if isnan(val)
            Cam=0;
        elseif idx==1
            Cam=1;
        elseif idx==2
            Cam=2;
        elseif idx==3
            Cam=3;
        end
    end
end
end