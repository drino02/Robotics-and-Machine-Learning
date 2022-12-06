function [x, frames] = dirkin(q)
%DIRKIN Direct Kineematics of 4DOF planar robot
%   Input:
%       q - destination robot configuration (in degrees)
%   Output:
%       x - gripper position vector

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

% DH Table in Matrix form (theta, d, a, alpha)
DH_Mat = [q(1) 0 a1 0;
    q(2) 0 a2 0;
    q(3) 0 a3 0;
    q(4) 0 a4 0;
    q(5) 0 a5 0;
    q(6) 0 a6 0;
    q(7) 0 a7 0;
    q(8) 0 a8 0];
    

% extract row size of DH_Mat
m = size(DH_Mat,1);

% initialie frames
frames = [];

temp = [];

% lambda function to convert compute homogeneous matrix
HMat = @(theta, d, a, alpha)...
    [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);...
    sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);...
    0 sin(alpha) cos(alpha) d;...
    0 0 0 1];

% convert each row of DH_Mat to corrresponding homogeneous matrix
for i = 1:m
    Row = DH_Mat(i,:);
    cur = HMat(Row(1),Row(2),Row(3),Row(4));
    if isempty(frames)
        frames(:,:,i) = cur;
    else
        frames(:,:,i) = temp*cur;
    end
    temp = frames(:,:,i);
end

x = frames(1:2,4,m);

end

