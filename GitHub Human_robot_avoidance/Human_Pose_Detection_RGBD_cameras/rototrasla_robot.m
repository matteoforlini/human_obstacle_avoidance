
function [c1_ass]=rototrasla_robot(coordinate3d_double,camera_id_int)

data1_coordinate=coordinate3d_double;

if isempty(data1_coordinate)  %fill a Nan value matrix if is not detected any skeleton in the scene
    c1_ass(1,:)=[0:1:32];
    c1_ass(2:5,:)=NaN(4,33);
else

%Reorganize the data1_coordinate matrix: rows correspond to id, visibility, x, y, z, and each column should represent a single body joint.
id_row=[];
for i=1:5:(size(data1_coordinate,2)-4)
    c=data1_coordinate(:,i);
    id_row=[id_row,c];
end

vis_row=[];
for i=2:5:(size(data1_coordinate,2)-3)
    c=data1_coordinate(:,i);
    vis_row=[vis_row,c];
end

c1x_row=[];
for i=3:5:(size(data1_coordinate,2)-2)
    c=data1_coordinate(:,i);
    c1x_row=[c1x_row,c];
end

c1y_row=[];
for i=4:5:(size(data1_coordinate,2)-1)
    c=data1_coordinate(:,i);
    c1y_row=[c1y_row,c];
end

c1z_row=[];
for i=5:5:size(data1_coordinate,2)
        c=data1_coordinate(:,i);
        c1z_row=[c1z_row,c];
end

c1=[];
c1(1,:)=id_row;
c1(2,:)=vis_row;
c1(3,:)=c1x_row;
c1(4,:)=c1y_row;
c1(5,:)=c1z_row;

%1,2,3 are camera reference frame, r is robot reference frame
if camera_id_int==46122250287
    T1r_opt = load('T1r_opt.mat');
    Tcam_r_opt = T1r_opt.T1r_opt; %Tcam_r_opt directly links the camera position relative to the robot base. The inverse of this matrix is needed to transform points from the camera's reference system to the robot's. 
elseif  camera_id_int==46122251438 
    T2r_opt = load('T2r_opt.mat');
    Tcam_r_opt = T2r_opt.T2r_opt;

elseif camera_id_int==46122250173
    T3r_opt = load('T3r_opt.mat'); 
    Tcam_r_opt = T3r_opt.T3r_opt;
end


Point1=[];
c1_ass=[];
for i=1:size(c1,2)
    for j=1:(size(c1,1)-2)
        P1=c1(j+2,i);
        Point1(j)=P1;%mm
    end
    Point1_sist_robot=inv(Tcam_r_opt)*[Point1';1];
    c1_ass(1,i)=c1(1,i);
    c1_ass(2,i)=c1(2,i);
    c1_ass(3,i)=Point1_sist_robot(1);
    c1_ass(4,i)=Point1_sist_robot(2);
    c1_ass(5,i)=Point1_sist_robot(3);
end
end
end



