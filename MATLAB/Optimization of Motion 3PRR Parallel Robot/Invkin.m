function q = Invkin(p, theta,r,b, ea)
%INVKIN Summary of this function goes here
%   Input:
%       theta - orientation of end-effector wrt to world
%       r - constant distance of moving platform's vertex to the center of
%           the moving platform
%       b - contant distance of the  vertex of the base from the center o
%           the base platform
%       ea - length of segment ea
%   Output:
%       q - current configuration

% moving platform
rot_p_w = rotz(theta); % rotation of end-effector wrt to the world
rot_p_w = rot_p_w(1:2,1:2); % extract the cols and rows corresponding 
                            % only x and y
a1 = p + rot_p_w*([r*cosd(-30); r*sind(-30)]);
a2 = p + rot_p_w*([r*cosd(90); r*sind(90)]);
a3 = p + rot_p_w*([r*cosd(-150); r*sind(-150)]);

% static platform
b1 = [b*cosd(-30); b*sind(-30)];
b2 = [b*cosd(90); b*sind(90)];
b3 = [b*cosd(-150); b*sind(-150)];

% e vectors
beta = acos(((norm(a1-b3)^2) + (norm(b1-b3)^2) - (norm(a1-b1)^2))/...
    (2*norm(a1-b3)*norm(b1-b3)));
phi = acos(((norm(a1-b1)^2) + (norm(b1-b3)^2) - (norm(a1-b3)^2))/...
    (2*norm(a1-b1)*norm(b1-b3)));
gamma = asin((norm(a1-b3)*sin(beta))/ea);

e1 = b1+(norm(a1-b1)*cos(phi)+ea*cos(gamma))*(b3-b1)/norm(b3-b1);
e2 = b2+(norm(a2-b2)*cos(phi)+ea*cos(gamma))*(b1-b2)/norm(b1-b2);
e3 = b3+(norm(a3-b3)*cos(phi)+ea*cos(gamma))*(b2-b3)/norm(b2-b3);

% configuration
q = [norm(e1-b1);
     norm(e2-b2);
     norm(e3-b3)];

end

