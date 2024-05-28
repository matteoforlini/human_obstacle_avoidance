function [k,d,x]=dis_link(robot_point_g,o)
% link
link_1=robot_point_g(:,2)-robot_point_g(:,1);
link_2=robot_point_g(:,3)-robot_point_g(:,2);
link_3a=robot_point_g(:,4)-robot_point_g(:,3);
link_3b=robot_point_g(:,5)-robot_point_g(:,4);
link_4=robot_point_g(:,6)-robot_point_g(:,5);
link_5=robot_point_g(:,7)-robot_point_g(:,6);
link_6=robot_point_g(:,8)-robot_point_g(:,7);
link=[link_1 link_2 link_3a link_3b link_4 link_5 link_6];
int=zeros(size(o,2),size(link,2));
distance=zeros(size(o,2),size(link,2));
d=zeros(size(link,2),1);
k=zeros(size(link,2),1);
x=zeros(size(link,2),1);
for n=1:size(o,2)
    for i=1:size(link,2)
        d_p=o(:,n)-robot_point_g(:,i);
        d_d=o(:,n)-robot_point_g(:,i+1);
        
        c_a=(dot(d_p,link(:,i)))/(norm(d_p)*norm(link(:,i)));
        c_b=(dot(d_d,-link(:,i)))/(norm(d_d)*norm(link(:,i)));
        
        if c_a >= 0 && c_b >=0
            distance(n,i)=norm(cross(link(:,i),d_d))/norm(link(:,i));
            int(n,i)=norm(d_p)*c_a/norm(link(:,i));   
        elseif c_a < 0 && c_b > 0
            distance(n,i)=norm(d_p);
            int(n,i)=0;
        elseif c_b < 0 && c_a > 0
            distance(n,i)=norm(d_d);
            int(n,i)=1;
        end
    end
end

for s=1:size(link,2)
    [d(s), k(s)]=min(distance(:,s));
    x(s)=int(k(s),s);
end


