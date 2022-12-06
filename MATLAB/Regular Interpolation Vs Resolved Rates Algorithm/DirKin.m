function [frames, p, R] = DirKin(q)
%DIRKIN Direct Kinematics of PUMA560
%   Input:
%       q - configuration vector in RADIANS
%   Output:
%       frames - 6-dimensional matrix containing all homogeneous
%                transformation matrices from the first to the last link
%       p - position of the tooltip (marker)
%       R - orientation of the tooltip

% DH Table in Matrix form
DH_Mat = [q(1) 0.67183 0 pi/2;
    q(2) 0.1501 0.4318 0;
    q(3) 0 -0.0203 pi/2;
    q(4) 0.4331 0 pi/2;
    q(5) 0 0 -pi/2;
    q(6) 0.0558 0 0];

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

Hmat_tooltip = frames(:,:,m)*HMat(0,0.12,0,0);
p = Hmat_tooltip(1:3,4);
R = Hmat_tooltip(1:3,1:3);
end

