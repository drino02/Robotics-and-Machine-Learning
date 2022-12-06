
% B= 3x3 matrix containing vertices of base platfrom in columns
% A = 3x3 matrix containing vertices of moving platform in columns
% E = 3x3 matrix constaining coordinates of elbows

function []=Draw_Robot(B,A,E)

%%center of triangle 
t=sum(A,2)/3; %the center of the equilateral triangular moving platfrom

%% base triangle 
h=patch(B(1,:),B(2,:),B(3,:)); % create a red patch with handle h
set(h,'edgecolor','k','linewidth',2,'FaceColor','w','facealpha',0); % set the edge color for the patch

%% moving platform
h=patch(A(1,:),A(2,:),A(3,:)); % create a red patch with handle h
set(h,'edgecolor','g','linewidth',2,'FaceColor','m','facealpha',1); % set the edge color for the patch

%% lines for the base slides and the links 
X=[B(1,:), E(1,:); ...
  E(1,:) , A(1,:)];

Y=[B(2,:), E(2,:); ...
  E(2,:) , A(2,:)];

Z=[B(3,:), E(3,:); ...
  E(3,:) , A(3,:)];

h=line(X,Y,Z);

set(h(1),'Color','r','linewidth',4); 
set(h(2),'Color','r','linewidth',4); 
set(h(3),'Color','r','linewidth',4); 

set(h(4),'Color','b','linewidth',2); 
set(h(5),'Color','b','linewidth',2); 
set(h(6),'Color','b','linewidth',2); 

%draw coordinate systems
base_width=norm(B(:,2)-B(:,1));
draw_coordinate_system(base_width*0.2,rotd([0,0,1],0),[0,0,0]','rgb','w');
platfrom_width=norm(A(:,2)-A(:,1));
theta=atan2( A(2,1)-A(2,3),A(1,1)-A(1,3) )*180/pi; %angle of moving platform 
draw_coordinate_system(platfrom_width*0.5,rotd([0,0,1],theta),t,'rgb','m');

%% draw the carriages for the sliding joints
a=platfrom_width/2; %since the moving platform width is 2a
carriage_width=a;
carriage_height=a/2;

Lower_right=[carriage_width/2;-carriage_height/2;0]*ones(1,3);
Upper_right=[carriage_width/2;+carriage_height/2;0]*ones(1,3);
Lower_left=[-carriage_width/2;-carriage_height/2;0]*ones(1,3);
Upper_left=[-carriage_width/2;+carriage_height/2;0]*ones(1,3);

eta=[180,-60,60]*pi/180; %direction of the sliders in world frame
Rotz=@(t) [cos(t), -sin(t), 0;...
    sin(t), cos(t), 0;...
    0,0,1];
%rotate the carriages
for i=1:1:3
    Lower_right(:,i)= Rotz(eta(i))*Lower_right(:,i);
    Upper_right(:,i)= Rotz(eta(i))*Upper_right(:,i);
    Lower_left(:,i)= Rotz(eta(i))*Lower_left(:,i);
    Upper_left(:,i)= Rotz(eta(i))*Upper_left(:,i);
end;
%translate the carriages
Lower_right=E+Lower_right;
Upper_right=E+Upper_right;
Lower_left=E+Lower_left;
Upper_left=E+Upper_left;

for i=1:1:3
X=[Lower_right(1,i), Upper_right(1,i),Upper_left(1,i), Lower_left(1,i),Lower_right(1,i)];
Y=[Lower_right(2,i), Upper_right(2,i),Upper_left(2,i), Lower_left(2,i),Lower_right(2,i)];
Z=[Lower_right(3,i), Upper_right(3,i),Upper_left(3,i), Lower_left(3,i),Lower_right(3,i)];
h=patch(X,Y,Z,'y');
end;
%% draw base joint circles
drawcircle(a/6,B(:,1),20,'r');
drawcircle(a/6,B(:,2),20,'r');
drawcircle(a/6,B(:,3),20,'r');

%% draw elbow joint circles
drawcircle(a/6,E(:,1),20,'g');
drawcircle(a/6,E(:,2),20,'g');
drawcircle(a/6,E(:,3),20,'g');

%% draw platfrom joint circles
drawcircle(a/4,A(:,1),20,'g');
drawcircle(a/4,A(:,2),20,'g');
drawcircle(a/4,A(:,3),20,'g');
xlim([-0.4 0.4])
ylim([-0.3 0.5])


