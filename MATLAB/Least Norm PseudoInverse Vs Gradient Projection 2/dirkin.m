function [frames, x_gripper] = dirkin(q)
%DIRKIN Direct Kineematics of 4DOF planar robot
%   Input:
%       q - destination robot configuration (in radians)
%   Output:
%       x_gripper - gripper position vector

% constants
a = 1;
a1 = a;
a2 = a;
a3 = a;

% DH Table in Matrix form (theta, d, a, alpha)
DH_Mat = [0 0 q(1) 0;
            q(2) 0 a1 0;
            q(3) 0 a2 0;
            q(4) 0 a3 0];

% extract row size of DH_Mat
[m,n] = size(DH_Mat);

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

% gripper center offset to the last frame in meters
gripper_offset = 0.1;

frame_gripper = frames(:,:,m)*HMat(0,0,gripper_offset,0);
x_gripper = frame_gripper(1:2,4);

end

