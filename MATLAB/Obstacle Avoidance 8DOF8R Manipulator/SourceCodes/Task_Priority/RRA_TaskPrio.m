function [q_dot,delta_p] = RRA_TaskPrio(pc,pd,obs,J1,J2,closest2obs)
%RRA_TASKPRIO RESOLVED RATES ALGORIHTM FOR TASK PRIORITY RESOLUTION
%   Input:
%       pc - current position coordinates
%       pd - destination position coordinates
%       obs - obstacle coordinates
%       J1 - Jacobian of the first task
%       J2 - Jacobian of the secondary task
%       closest2obs - position coordinates of the closest point in the arm
%               wrt to the obstacle
%       radius - radius of the region of influence of the obstacle
%   Output:
%       qd_dot - rate of change of configuration
%       delta_p - currrent position error norm

% tuning parameters
alpha1 = 10;%1/norm(closest2obs-obs);
alpha2 = 1;

% For the task priority resolution formula
I_size = size(J1,2); 
J1_tilda = J2*(eye(I_size,I_size)-pinv(J1)*J1);

% resolved rates algorithm parameters (translational)
eps_p = 0.001;
vmin = 0.001;
vmax = 0.05;
lambda_p = 50;

% error parameters
delta_p = norm(pd-pc);

% normalized position error vector
nhat = (pd - pc)/norm(pd-pc);

% conditions for vmag (linear velocity magnitude)
if  delta_p/eps_p >  lambda_p
    vmag = vmax;
else
    vmag = vmin + ((vmax-vmin)*(delta_p-eps_p))/(eps_p*(lambda_p-1));
end
x_dot = vmag*nhat;

% direction of closest point to obstacle
nhat2 = -(obs'-closest2obs)/norm(obs' - closest2obs);

% velocity of closest point away from the obstacle
x2_dot = vmag*nhat2;

q_dot = pinv(J1)*x_dot+alpha1*pinv(J1_tilda)*(alpha2*x2_dot-J2*pinv(J1)*x_dot);

end

