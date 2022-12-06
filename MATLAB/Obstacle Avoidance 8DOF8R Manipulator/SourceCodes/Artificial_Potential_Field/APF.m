function [q_dot,delta_p] = APF(pc,pd,obs,J1,J2,closest2obs,rho_o)
% APF ARTIFICIAL POTENTIAL FIELD ALGORITHM
%   Input:
%       pc - current position coordinates
%       pd - destination position coordinates
%       obs - obstacle coordinates
%       J1 - Jacobian of the first task
%       J2 - Jacobian of the secondary task
%       closest2obs - position coordinates of the closest point in the arm
%               wrt to the obstacle
%       rho_o - radius of the region of influence of the obstacle
%   Output:
%       qd_dot - rate of change of configuration
%       delta_p - currrent position error norm

% tuning parameters
zeta = 1; % gain of the attractive force
eta = 1; % gain of the repulsive force
alpha = 0.005;% gain (<1)

% error parameters
delta_p = norm(pd-pc);

% Force of attraction formula (negative gradient of U_att)
% d is always assumed to be infinite, hence,
F_att = -zeta*(pc-pd);

rho_p = norm(closest2obs-obs);
rho_p_grad = (closest2obs-obs)/norm(closest2obs-obs);

if rho_p <= rho_o
    F_rep = (eta*((1/rho_p)-(1/rho_o))/(rho_p^2))*rho_p_grad;
else
    F_rep = [0 0]';
end

tau = J1'*F_att+J2'*F_rep;
q_dot = alpha*tau/norm(tau);

end

