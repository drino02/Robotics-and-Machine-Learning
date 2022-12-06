function [x, frames] = dirkin(q)
%DIRKIN Direct Kineematics of 4DOF planar robot
%   Input:
%       q - destination robot configuration (in degrees)
%   Output:
%       x - gripper position vector

% constants
a = 1;
a1 = a;
a2 = (3/4)*a;
a3 = (1/2)*a;
a4 = (1/4)*a;

% DH Table in Matrix form (theta, d, a, alpha)
DH_Mat = [q(1) 0 a1 0;
    q(2) 0 a2 0;
    q(3) 0 a3 0;
    q(4) 0 a4 0];

% extract row size of DH_Mat
[m,n] = size(DH_Mat);

% initialie frames
frames = [];

temp = [];

% lambda function to convert compute homogeneous matrix
HMat = @(theta, d, a, alpha)...
    [cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha) a*cosd(theta);...
    sind(theta)  cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha) a*sind(theta);...
    0 sind(alpha) cosd(alpha) d;...
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

