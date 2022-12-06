clc;
close all;
clear ;

load('start.mat');

a  = 0.1; % distance of one base vertex to another
b = (3*a/cosd(30)); % distance of one base vertex to the center of the base platform
r = (a/cosd(30));
ea = 0.3;
eps = 0.01; % error tolerance
dt = 1;

% goal
theta_goal = 0;
radius = a;
pgoals = [];
for i = 0:360
    pgoals(1:2,end+1) = [radius*cosd(i); radius*sind(i)];
end

% step sizes
p_dot = 0.01;
theta_dot_mag = 0.5;

% static platform
b1 = [b*cosd(-30); b*sind(-30)];
b2 = [b*cosd(90); b*sind(90)];
b3 = [b*cosd(-150); b*sind(-150)];


% current values
pcur = [0;0];
theta_cur = (acos(dot(A_in_w(1:2,2)-pcur, B(1:2,2))/...
    (norm(A_in_w(1:2,2)-pcur)*norm(B(1:2,2)))))*180/pi;

% for movieVector
ind = 1;

%for trajectory plot
trace_x =[];
trace_y = [];

%% for animation
fig1 = figure(1);

for i = 1:length(pgoals)
    pgoal = pgoals(1:2,i);
    while true
        clf
        q = Invkin(pcur,theta_cur,r,b,ea) % for output visualization only
        Draw_Robot(B,A_in_w,E);
        if size(trace_x,2) > 1
            line(trace_x,trace_y,'Color','b', 'LineStyle','--');
        end
        delta_p = norm(pcur - pgoal);
        delta_theta = norm(theta_cur - theta_goal);

        if delta_p <= eps
            break;
        end

        if delta_p > eps
            nhat = (pgoal-pcur)/norm(pgoal-pcur);
            pcur = pcur + nhat*p_dot;

            % moving platform
            rot_p_w = rotz(theta_cur); % rotation of end-effector wrt to the world
            rot_p_w = rot_p_w(1:2,1:2); % extract the cols and rows corresponding
            % only x and y
            a1 = pcur + rot_p_w*([r*cosd(-30); r*sind(-30)]);
            a2 = pcur + rot_p_w*([r*cosd(90); r*sind(90)]);
            a3 = pcur + rot_p_w*([r*cosd(-150); r*sind(-150)]);

            % e vectors
            beta1 = acos(((norm(a1-b3)^2) + (norm(b1-b3)^2) - (norm(a1-b1)^2))/...
                (2*norm(a1-b3)*norm(b1-b3)));
            phi1 = acos(((norm(a1-b1)^2) + (norm(b1-b3)^2) - (norm(a1-b3)^2))/...
                (2*norm(a1-b1)*norm(b1-b3)));
            gamma1 = asin((norm(a1-b3)*sin(beta1))/ea);

            beta2 = acos(((norm(a2-b1)^2) + (norm(b1-b2)^2) - (norm(a2-b2)^2))/...
                (2*norm(a2-b1)*norm(b1-b2)));
            phi2 = acos(((norm(a2-b2)^2) + (norm(b1-b2)^2) - (norm(a2-b1)^2))/...
                (2*norm(a2-b2)*norm(b1-b2)));
            gamma2 = asin((norm(a2-b1)*sin(beta2))/ea);

            beta3 = acos(((norm(a3-b2)^2) + (norm(b2-b3)^2) - (norm(a3-b3)^2))/...
                (2*norm(a3-b2)*norm(b2-b3)));
            phi3 = acos(((norm(a3-b3)^2) + (norm(b2-b3)^2) - (norm(a3-b2)^2))/...
                (2*norm(a3-b3)*norm(b2-b3)));
            gamma3 = asin((norm(a3-b2)*sin(beta3))/ea);

            e1 = b1+(norm(a1-b1)*cos(phi1)+ea*cos(gamma1))*(b3-b1)/norm(b3-b1);
            e2 = b2+(norm(a2-b2)*cos(phi2)+ea*cos(gamma2))*(b1-b2)/norm(b1-b2);
            e3 = b3+(norm(a3-b3)*cos(phi3)+ea*cos(gamma3))*(b2-b3)/norm(b2-b3);

            % update current values
            theta_cur = acos(dot(a2-pcur,b2)/(norm(a2-pcur)*norm(b2)));
            A_in_w = [a1 a2 a3];
            A_in_w(end+1,:) = [0 0 0];
            E = [e1 e2 e3];
            E(end+1,:) = [0 0 0];
            trace_x(1,end+1) = pcur(1);
            trace_y(1,end+1) = pcur(2);
        end
        
        if delta_theta > eps
        nhat = (theta_goal-theta_cur)/norm(theta_goal-theta_cur);
        theta_cur = theta_cur + nhat*theta_dot_mag*dt;

        % moving platform
        rot_p_w = rotz(theta_cur); % rotation of end-effector wrt to the world
        rot_p_w = rot_p_w(1:2,1:2); % extract the cols and rows corresponding
        % only x and y
        a1 = pcur + rot_p_w*([r*cosd(-30); r*sind(-30)]);
        a2 = pcur + rot_p_w*([r*cosd(90); r*sind(90)]);
        a3 = pcur + rot_p_w*([r*cosd(-150); r*sind(-150)]);

        % e vectors
        beta1 = acos(((norm(a1-b3)^2) + (norm(b1-b3)^2) - (norm(a1-b1)^2))/...
            (2*norm(a1-b3)*norm(b1-b3)));
        phi1 = acos(((norm(a1-b1)^2) + (norm(b1-b3)^2) - (norm(a1-b3)^2))/...
            (2*norm(a1-b1)*norm(b1-b3)));
        gamma1 = asin((norm(a1-b3)*sin(beta1))/ea);

        beta2 = acos(((norm(a2-b1)^2) + (norm(b1-b2)^2) - (norm(a2-b2)^2))/...
            (2*norm(a2-b1)*norm(b1-b2)));
        phi2 = acos(((norm(a2-b2)^2) + (norm(b1-b2)^2) - (norm(a2-b1)^2))/...
            (2*norm(a2-b2)*norm(b1-b2)));
        gamma2 = asin((norm(a2-b1)*sin(beta2))/ea);

        beta3 = acos(((norm(a3-b2)^2) + (norm(b2-b3)^2) - (norm(a3-b3)^2))/...
            (2*norm(a3-b2)*norm(b2-b3)));
        phi3 = acos(((norm(a3-b3)^2) + (norm(b2-b3)^2) - (norm(a3-b2)^2))/...
            (2*norm(a3-b3)*norm(b2-b3)));
        gamma3 = asin((norm(a3-b2)*sin(beta3))/ea);

        e1 = b1+(norm(a1-b1)*cos(phi1)+ea*cos(gamma1))*(b3-b1)/norm(b3-b1);
        e2 = b2+(norm(a2-b2)*cos(phi2)+ea*cos(gamma2))*(b1-b2)/norm(b1-b2);
        e3 = b3+(norm(a3-b3)*cos(phi3)+ea*cos(gamma3))*(b2-b3)/norm(b2-b3);

        % update current values
        A_in_w = [a1 a2 a3];
        A_in_w(end+1,:) = [0 0 0];
        E = [e1 e2 e3];
        E(end+1,:) = [0 0 0];
    end
        movieVector(ind) = getframe(fig1, [30 10 500 400]);
        ind = ind + 1;
    end
    
end

% create video file
myWriter = VideoWriter('circle', 'MPEG-4');
myWriter.FrameRate = 10;
open(myWriter);
writeVideo(myWriter,movieVector);
close(myWriter);