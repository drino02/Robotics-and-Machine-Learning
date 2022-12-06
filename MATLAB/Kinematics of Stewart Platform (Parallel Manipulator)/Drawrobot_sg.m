function Drawrobot_sg(A_wcs,B),
% This function draws stwart gough robot based on given 
% locations of its spherical joints

% draw moving platform
h_t=patch(A_wcs(1,:),A_wcs(2,:),A_wcs(3,:),'r');
set(h_t,'edgecolor','g','linewidth',1);
%set(h_t); %displays the current settings for the patch handle
alpha(h_t,0.5);

% draw base platform
h_b=patch(B(1,:),B(2,:),B(3,:),'y');
set(h_b,'edgecolor','w','linewidth',1);
%set(h_b); %displays the current settings for the patch handle

%draw legs
x_start=A_wcs(1,:);
x_finish=B(1,:);

y_start=A_wcs(2,:);
y_finish=B(2,:);

z_start=A_wcs(3,:);
z_finish=B(3,:);

h=line([x_start;x_finish],[y_start;y_finish],[z_start;z_finish]);
set(h,'linewidth',3,'color','k');

%calculate the coordinate system of moving platform from their coordinates
% r_b=norm(B(:,1)); %radius of base
% r_t=norm(A_wcs(:,1)); %radius of top platform
r_b=150; %radius of base
r_t=100; %radius of top platform
t=(A_wcs(:,1)+A_wcs(:,3)+A_wcs(:,5))/3; %geometric average gives center of moving platform

z_a=cross(A_wcs(:,1)-t,A_wcs(:,3)-t);
z_a=z_a/norm(z_a);
x_a=A_wcs(:,6)+1/2*(A_wcs(:,1)-A_wcs(:,6))-t;
x_a=x_a/norm(x_a);
y_a=cross(z_a,x_a);

R=[x_a,y_a,z_a];
%------------------------
% draw coordinate systems
draw_coordinate_system(r_b,eye(3),[0,0,0]','rgb'); %draw world coordinate system
draw_coordinate_system(r_t,R,t,'rgc'); %draw world coordinate system

end


