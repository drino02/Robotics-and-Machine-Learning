function [qd_dot,delta_p] = RRA_GradProj(q,q0,pc,pd)
%RRA_Grd Proj Resolved Rates Algorithm via Gradient Projection
%   Input:
%       q - current configuration
%       q0 - home position configuration
%       pc - current position
%       pd - destination position
%   Output:
%       qd_dot - rate of change of configuration
%       delta_p - currrent position error norm

% gripper center offset to the last frame in meters
offset = 0.1;

% for the Jacobian
J = [1 -sin(q(2))-sin(q(2)+q(3))-(1+offset)*sin(q(2)+q(3)+q(4))...
    -sin(q(2)+q(3))-(1+offset)*sin(q(2)+q(3)+q(4)) -(1+offset)*sin(q(2)+q(3)+q(4));
    0 cos(q(2))+cos(q(2)+q(3))+(1+offset)*cos(q(2)+q(3)+q(4))...
    cos(q(2)+q(3))+(1+offset)*cos(q(2)+q(3)+q(4)) (1+offset)*cos(q(2)+q(3)+q(4))];

% resolved rates algorithm parameters (translational)
eps_p = 0.001;
vmin = 0.01;
vmax = 0.1;
lambda_p = 7;

% error parameters
delta_p = sqrt((pd-pc)'*(pd-pc));

% normalized position error vector
nhat = (pd - pc)/norm(pd-pc);

% conditions for vmag (linear velocity magnitude)
if  delta_p/eps_p >  lambda_p
    vmag = vmax;
else
    vmag = vmin + ((vmax-vmin)*(delta_p-eps_p))/(eps_p*(lambda_p-1));
end

% compute linear velocity component of error vector
pd_dot = vmag*nhat;

xd_dot = pd_dot; %[pd_dot']';

% define alpha
alpha = 1;

% redundancy resolution via gradient projection
qd_dot = pinv(J)*xd_dot+(eye(4,4)-pinv(J)*J)*alpha*(q-q0); 
end

