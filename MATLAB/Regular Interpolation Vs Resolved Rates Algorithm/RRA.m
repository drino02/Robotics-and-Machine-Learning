function [qd_dot,delta_p,delta_o] = RRA(frames,pc,pd,Rd)
%RRA Resolved Rates Algorithm
%   Input:
%       frames - 6D matrix output of DirKin fcn representing the 6
%       successive homogeneous transformations form frame 0 to 6
%       pc - current position
%       pd - destination position
%       Rd - destination rotation matrix
%   Output:
%       qd_dot - rate of change of configuration
%       delta_p - currrent position error norm
%       delta_o - currrent orientation error norm

% currrent rotation
Rc = frames(1:3,1:3,6);

% form the Jacobian
n = length(frames);
J = [cross([0;0;1],pc); [0; 0; 1]];
for i = 1:n-1
    J(1:n,i+1) = [cross(frames(1:3,3,i),(pc-frames(1:3,4,i)));...
        frames(1:3,3,i)];  
end

% resolved rates algorithm parameters (translational)
eps_p = 0.001;
vmin = 0.03;
vmax = 0.1;
lambda_p = 5;

% resolved rates algorithm parameters (rotational)
eps_o = 0.0524;
omin = 0.0349;
omax = 0.1745;
lambda_o = 5;

% error parameters
delta_p = sqrt((pd-pc)'*(pd-pc));
Re = Rd*Rc';
theta_e = acos((trace(Re)-1)/2);
m_e = (1/(2*sin(theta_e)))*[Re(3,2)-Re(2,3); Re(1,3)-Re(3,1); Re(2,1)-Re(1,2)];
delta_o = theta_e;

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

%conditions for omag (angular velocity magnitude)
if  delta_o/eps_o >  lambda_o
    omag = omax;
else
    omag = omin + ((omax-omin)*(delta_o-eps_o))/(eps_o*(lambda_o-1));
end

% compute angular velocity component of error vector
omega_d = omag*m_e;

xd_dot = [pd_dot' omega_d']';
qd_dot = pinv(J)*xd_dot;
end

