function [] = DrawRobot_HW3p3(Frames)
%UNTITLED2 Summary of this function goes here
%o1 to o4 are the location vectors of the frame origins
o1=Frames(1:3,4,1);
o2=Frames(1:3,4,2);
o3=Frames(1:3,4,3);
o4=Frames(1:3,4,4);
o5=Frames(1:3,4,5);

%---- drawing the links -----
x_matrix=[[o1(1,1);o2(1,1)],[o2(1,1);o3(1,1)],[o3(1,1);o4(1,1)],[o4(1,1);o5(1,1)]];
y_matrix=[[o1(2,1);o2(2,1)],[o2(2,1);o3(2,1)],[o3(2,1);o4(2,1)],[o4(2,1);o5(2,1)]];

h=line(x_matrix,y_matrix);
set(h,'linewidth',3);
set(h(1),'color','r');
set(h(2),'color','k');
set(h(3),'color','b');
%----- drawing the base ------
cart_width=norm(o2-o3,2);
cart_bottom_left=o1+[-cart_width/2;-cart_width/6;0];
cart_bottom_right=o1+[cart_width/2;-cart_width/6;0];
cart_top_right=o1+[cart_width/2;0;0];
cart_top_left=o1+[-cart_width/2;0;0];

cart=[cart_bottom_left,cart_bottom_right,cart_top_right,cart_top_left];
patch(cart(1,:),cart(2,:),[205,183,158]/255);

%------
cart_wheel_radius=cart_width/12;
left_wheel_origin=cart_bottom_left+cart_width/6*[1;0;0];
right_wheel_origin=cart_bottom_right-cart_width/6*[1;0;0];
drawcircle(cart_wheel_radius,left_wheel_origin); 
drawcircle(cart_wheel_radius,right_wheel_origin); 

axis_length=norm(o2-o3,2)/3;
draw_coordinate_system(axis_length,Frames(1:3,1:3,1),o1,'rgb','1');
draw_coordinate_system(axis_length,Frames(1:3,1:3,2),o2,'rgb','2');
draw_coordinate_system(axis_length,Frames(1:3,1:3,3),o3,'rgb','3');
draw_coordinate_system(axis_length,Frames(1:3,1:3,4),o4,'rgb','4');
draw_coordinate_system(axis_length,Frames(1:3,1:3,5),o5,'rgb','5');

end

