function [qd_dot,delta_p] = RRA_WLS_PWF(q,pc,pd)
%RRA_WLS_PWF Resolved Rates Algorithm via Weighted Least Squares &
%   Potential Well Function
%
%   Input:
%       q - current configuration
%       pc - current position
%       pd - destination position
%   Output:
%       qd_dot - rate of change of configuration
%       delta_p - currrent position error norm

% gripper center offset to the last frame in meters
offset = 0.1;

% form the Jacobian
J = [1 1 -sin(q(3))-sin(q(3)+q(4))-(1+offset)*sin(q(3)+q(4)+q(5))...
    -sin(q(4)+q(4))-(1+offset)*sin(q(3)+q(4)+q(5)) -(1+offset)*sin(q(3)+q(4)+q(5));
    0 0 cos(q(3))+cos(q(3)+q(4))+(1+offset)*cos(q(3)+q(4)+q(5))...
    cos(q(3)+q(4))+(1+offset)*cos(q(3)+q(4)+q(5)) (1+offset)*cos(q(3)+q(4)+q(5))];

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

xd_dot = pd_dot; 

% WLS with potential well function
q2max = 0.1;
q2min = -0.1;
q2mid = (q2max+q2min)/2; 
w_q2 = 0.1 + abs((((q2max-q2min)^2)*(q(2)-q2mid))/(2*((q2max-q(2))^2)*((q(2)-q2min)^2)));
W = [1 0 0 0 0; 0 w_q2 0 0 0; 0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 1];
qd_dot = pinv(W)*J'*pinv(J*pinv(W)*J')*xd_dot;

end

