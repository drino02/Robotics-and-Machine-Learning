function [frames, p] = DirKin(a, q)
%DIRKIN Direct Kinematics of PUMa(5)60
%   Input:
%       q      - configuration vector in degrees
%   Output:
%       frames - 4x4xn matrix containing all homogeneous transformation 
%                matrices from the first to the last joint
%       p      - position of the end-effector

% DH table in matrix form
DH_Mat = [q(1) a(1) 0   90;
          q(2)  0  a(2) 0;
          q(3)  0  a(3) 0;
          q(4)  0  a(4) 0;
          q(5)  0  a(5) 0];

% Extract row count of DH_Mat
m = size(DH_Mat, 1);

% Anonymous function to compute the transformation matrix
Trans_Mat = @(theta, d, a, alpha)...
    [cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha) a*cosd(theta);...
    sind(theta)  cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha) a*sind(theta);...
    0 sind(alpha) cosd(alpha) d;...
    0 0 0 1];

% Initialize frames
frames = [];

% Initialize container for the previous transformation matrix
trans_mat_prev = [];

% Convert and derive each succeeding transformation matrix 
% from the first joint to the last
for i = 1:m
    DH_Mat_Row = DH_Mat(i,:);  % current row in the DH table
    trans_mat_cur = Trans_Mat(DH_Mat_Row(1), DH_Mat_Row(2), ...
        DH_Mat_Row(3), DH_Mat_Row(4));  % current transformation matrix
    if isempty(frames)
        frames(:,:,i) = trans_mat_cur;
    else
        frames(:,:,i) = trans_mat_prev*trans_mat_cur;
    end
    trans_mat_prev = frames(:,:,i);
end

p = frames(1:3, 4, m);

end

