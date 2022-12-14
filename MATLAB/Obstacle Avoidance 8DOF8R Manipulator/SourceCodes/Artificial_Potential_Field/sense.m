function [J2,c] = sense(q,obs,frames,radius)
%SENSE Range Sensor
%   Input
%       q - current configuration
%       obs - obstacle cooridnates
%       frames - frames of the current q (output form dirkin)
%       radius - radius of the region of influence of the obstacle
%   Output
%       J2 - secondary Jacobian
%       c - coordinates of the closest point in the arm to the obstacle

% define resolution of arm points
dx = 0.01;

% link lengths
% constants
a = 1;
a1 = a;
a2 = a;
a3 = a;
a4 = a;
a5 = a;
a6 = a;
a7 = a;
a8 = a;

% startpoints of each link
l1_strt_pt = [0 0];
l2_strt_pt = [frames(1:2,4,1)];
l3_strt_pt = [frames(1:2,4,2)];
l4_strt_pt = [frames(1:2,4,3)];
l5_strt_pt = [frames(1:2,4,4)];
l6_strt_pt = [frames(1:2,4,5)];
l7_strt_pt = [frames(1:2,4,6)];
l8_strt_pt = [frames(1:2,4,7)];
l_strt_pts = {l1_strt_pt l2_strt_pt l3_strt_pt l4_strt_pt l5_strt_pt ...
              l6_strt_pt l7_strt_pt l8_strt_pt};

% Jacobian formula for points along each link
J_l1 = @(len_o) [-len_o*sin(q(1)) 0 0 0 0 0 0 0;...
                  len_o*cos(q(1)) 0 0 0 0 0 0 0];
J_l2 = @(len_o) [-(len_o*sin(q(1)+q(2))+a1*sin(q(1))) ...
                 -len_o*sin(q(1)+q(2)) 0 0 0 0 0 0;...
                  (len_o*cos(q(1)+q(2))+a1*cos(q(1)))  ...
                  len_o*cos(q(1)+q(2)) 0 0 0 0 0 0];
J_l3 = @(len_o) [-(len_o*sin(q(1)+q(2)+q(3))+a2*sin(q(1)+q(2))+a1*sin(q(1))) ...
                 -(len_o*sin(q(1)+q(2)+q(3))+a2*sin(q(1)+q(2))) ...
                 -len_o*sin(q(1)+q(2)+q(3)) ...
                 0 0 0 0 0;...
                 (len_o*cos(q(1)+q(2)+q(3))+a2*cos(q(1)+q(2))+a1*cos(q(1)))  ...
                 (len_o*cos(q(1)+q(2)+q(3))+a2*cos(q(1)+q(2)))  ...
                 len_o*cos(q(1)+q(2)+q(3)) ...
                 0 0 0 0 0];
J_l4 = @(len_o) [-(len_o*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))+a2*sin(q(1)+q(2))+a1*sin(q(1))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))+a2*sin(q(1)+q(2))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4))) ...
                  0 0 0 0;...
                  (len_o*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))+a2*cos(q(1)+q(2))+a1*cos(q(1)))  ...
                  (len_o*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))+a2*cos(q(1)+q(2)))  ...
                  (len_o*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))) ...
                  (len_o*cos(q(1)+q(2)+q(3)+q(4))) ...
                  0 0 0 0];
J_l5 = @(len_o) [-(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))+a2*sin(q(1)+q(2))+a1*sin(q(1))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))+a2*sin(q(1)+q(2))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5))) ...
                  0 0 0;...
                  (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))+a2*cos(q(1)+q(2))+a1*cos(q(1))) ...
                  (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))+a2*cos(q(1)+q(2))) ...
                  (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))) ...
                  (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))) ...
                  (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5))) ...
                  0 0 0];

J_l6 = @(len_o) [-(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))+a2*sin(q(1)+q(2))+a1*sin(q(1))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))+a2*sin(q(1)+q(2))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*sin(q(1)+q(2)+q(3)+q(4)+q(5))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))) ...
                  0 0;...
                  (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))+a2*cos(q(1)+q(2))+a1*cos(q(1))) ...
                  (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))+a2*cos(q(1)+q(2))) ...
                  (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))) ...
                  (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))) ...
                  (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*cos(q(1)+q(2)+q(3)+q(4)+q(5))) ...
                  (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))) ...
                  0 0];

J_l7 = @(len_o) [-(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))+a2*sin(q(1)+q(2))+a1*sin(q(1))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))+a2*sin(q(1)+q(2))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*sin(q(1)+q(2)+q(3)+q(4)+q(5))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))) ...
                 0;...
                 (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))+a2*cos(q(1)+q(2))+a1*cos(q(1))) ...
                 (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))+a2*cos(q(1)+q(2))) ...
                 (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))) ...
                 (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))) ...
                 (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*cos(q(1)+q(2)+q(3)+q(4)+q(5))) ...
                 (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))) ...
                 (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))) ...
                 0];

J_l8 = @(len_o) [-(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7)+q(8))+a7*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))+a2*sin(q(1)+q(2))+a1*sin(q(1))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7)+q(8))+a7*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))+a2*sin(q(1)+q(2))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7)+q(8))+a7*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7)+q(8))+a7*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7)+q(8))+a7*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*sin(q(1)+q(2)+q(3)+q(4)+q(5))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7)+q(8))+a7*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7)+q(8))+a7*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))) ...
                 -(len_o*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7)+q(8)));...
                 (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7)+q(8))+a7*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))+a2*cos(q(1)+q(2))+a1*cos(q(1))) ...
                 (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7)+q(8))+a7*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))+a2*cos(q(1)+q(2))) ...
                 (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7)+q(8))+a7*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))) ...
                 (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7)+q(8))+a7*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))) ...
                 (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7)+q(8))+a7*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*cos(q(1)+q(2)+q(3)+q(4)+q(5))) ...
                 (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7)+q(8))+a7*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))) ...
                 (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7)+q(8))+a7*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))) ...
                 (len_o*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7)+q(8)))];

J_l = {J_l1 J_l2 J_l3 J_l4 J_l5 J_l6 J_l7 J_l8};

arm_points = [];

% all points along a1, dx apart
for i=0:dx:a1
    arm_points(end+1,:) = [i*cos(q(1)) i*sin(q(1)) 1];
end

% all points along a2, dx apart
for i=dx:dx:a2
    arm_points(end+1,:) = [i*cos(q(1)+q(2))+a1*cos(q(1)) ...
        i*sin(q(1)+q(2))+a1*sin(q(1)) 2];
end

% all points along a3, dx apart
for i=dx:dx:a3
    arm_points(end+1,:) = [i*cos(q(1)+q(2)+q(3))+a2*cos(q(1)+q(2))+a1*cos(q(1)) ...
        i*sin(q(1)+q(2)+q(3))+a2*sin(q(1)+q(2))+a1*sin(q(1)) 3];
end

% all points along a4, dx apart
for i=dx:dx:a4
    arm_points(end+1,:) = [i*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))+a2*cos(q(1)+q(2))+a1*cos(q(1)) ...
        i*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))+a2*sin(q(1)+q(2))+a1*sin(q(1)) 4];
end

% all points along a5, dx apart
for i=dx:dx:a5
    arm_points(end+1,:) = [i*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))+a2*cos(q(1)+q(2))+a1*cos(q(1)) ...
        i*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))+a2*sin(q(1)+q(2))+a1*sin(q(1)) 5];
end

% all points along a6, dx apart
for i=dx:dx:a6
    arm_points(end+1,:) = [i*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))+a2*cos(q(1)+q(2))+a1*cos(q(1)) ...
        i*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))+a2*sin(q(1)+q(2))+a1*sin(q(1)) 6];
end

% all points along a7, dx apart
for i=dx:dx:a7
    arm_points(end+1,:) = [i*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))+a2*cos(q(1)+q(2))+a1*cos(q(1)) ...
        i*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))+a2*sin(q(1)+q(2))+a1*sin(q(1)) 7];
end

% all points along a8, dx apart
for i=dx:dx:a8
    arm_points(end+1,:) = [i*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7)+q(8))+a7*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*cos(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*cos(q(1)+q(2)+q(3)+q(4)+q(5))+a4*cos(q(1)+q(2)+q(3)+q(4))+a3*cos(q(1)+q(2)+q(3))+a2*cos(q(1)+q(2))+a1*cos(q(1)) ...
        i*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7)+q(8))+a7*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6)+q(7))+a6*sin(q(1)+q(2)+q(3)+q(4)+q(5)+q(6))+a5*sin(q(1)+q(2)+q(3)+q(4)+q(5))+a4*sin(q(1)+q(2)+q(3)+q(4))+a3*sin(q(1)+q(2)+q(3))+a2*sin(q(1)+q(2))+a1*sin(q(1)) 8];
end


% test the point closest to the obstacle
closest2obs = [];

for i = 1:length(arm_points)
    if isempty(closest2obs) || ...
            (norm(closest2obs(1:2)-obs) > norm(arm_points(i,1:2)'-obs))
        closest2obs = arm_points(i,:)';
    end
end

% get link number for closest point to obs
l_no_1 = closest2obs(3);

if norm(closest2obs(1:2)-obs) <= radius
    J2 = J_l{l_no_1}(norm(closest2obs(1:2)-l_strt_pts{l_no_1}));
else
    J2 = zeros(2,length(q));
end

% coordinate of the closest point to the obstacle
c = closest2obs(1:2);




