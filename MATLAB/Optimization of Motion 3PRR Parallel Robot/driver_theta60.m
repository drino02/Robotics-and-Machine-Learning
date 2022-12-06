clc;
close all;
clear ;

% load('start.mat');
set_figure('white',[0,90],0.4*[-1,1,-0.6,1,-0.5,0.5],'x[m]','y[m]','z[m]','3PRR robot')
A_in_w = [[0.100000000000000;-0.057735026918963] ...
    [4.295939874609706e-17;0.115470053837925] ...
    [-0.100000000000000;-0.057735026918963]];
A_in_w(end+1,:)=[0 0 0];
B = [0.300000000000000,2.121150477449814e-17,-0.300000000000000;...
    -0.173205080756888,0.346410161513776,-0.173205080756888;...
    0,0,0];
E = [[-0.176887462097269;-0.173205080756888] ...
    [0.238443731048635;-0.066586495408748] ...
    [-0.061556268951365;0.239791576165636]];
E(end+1,:) = [0 0 0];
a  = 0.1; % distance of one base vertex to another
b = (3*a/cosd(30)); % distance of one base vertex to the center of the base platform
r = (a/cosd(30));
ea = 0.3;
eps = 0.01; % error tolerance
dt = 1;

% step sizes
p_dot = 0.01;
theta_dot_mag = 0.5;

% static platform
b1 = [b*cosd(-30); b*sind(-30)];
b2 = [b*cosd(90); b*sind(90)];
b3 = [b*cosd(-150); b*sind(-150)];

% goal
pgoal = [cosd(60)*a/2;sind(60)*a/2];
theta_goal = 60;

% current values
pcur = [0;0];
theta_cur = (acos(dot(A_in_w(1:2,2)-pcur, B(1:2,2))/...
    (norm(A_in_w(1:2,2)-pcur)*norm(B(1:2,2)))))*180/pi;


% for movieVector
ind = 1;

% for plot
con_nmbrs = [];
x_trace = [];
colors = {'r','g','b'};

%% for theta == 0
fig1 = figure(1);

while true
    clf
    q = Invkin(pcur,theta_cur,r,b,ea) % for output visualization only
    Draw_Robot(B,A_in_w,E);
    delta_p = norm(pcur - pgoal);
    delta_theta = norm(theta_cur - theta_goal);

    if delta_theta <= eps
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
        n1 = (e1-b1)/norm(e1-b1);
        n2 = (e2-b2)/norm(e2-b2);
        n3 = (e3-b3)/norm(e3-b3);
        break;
    end

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

    movieVector(ind) = getframe(fig1, [30 10 500 400]);
    ind = ind + 1;
end

while true
    clf
    q = Invkin(pcur,theta_cur,r,b,ea) % for output visualization only
    Draw_Robot(B,A_in_w,E);

   % inverse condition number
    s1 = cross([0;0;1],[(a1-pcur)/norm(a1-pcur);0]);
    s1 = s1(1:2,1);
    s2 = cross([0;0;1],[(a2-pcur)/norm(a2-pcur);0]);
    s2 = s2(1:2,1);
    s3 = cross([0;0;1],[(a3-pcur)/norm(a3-pcur);0]);
    s3 = s3(1:2,1);
    A = [-(e1-a1)'*n1 0 0; ...
        0 -(e2-a2)'*n2 0; ...
        0 0 -(e3-a3)'*n3];
    B_4J = [-(e1-a1)' -(e1-a1)'*s1;...
        -(e2-a2)' -(e2-a2)'*s2;...
        -(e3-a3)' -(e3-a3)'*s3];
    J = pinv(A)*B_4J;
    K_d = diag([10^5 10^5 10^5]);
    K = J'*K_d*J;
    C = 1/cond(K);

    % update global plot values
    x_trace(end+1) = pcur(1);
    con_nmbrs(end+1) = C;

    delta_p = norm(pcur - pgoal);
    delta_theta = norm(theta_cur - theta_goal);

    if delta_p <= eps
        break;
    end

    a1 = [];
    a2 = [];
    a3 = [];
    e1 = [];
    e2 = [];
    e3 =[];
    n1 = [];
    n2 = [];
    n3 =[];

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
    A_in_w = [a1 a2 a3];
    A_in_w(end+1,:) = [0 0 0];
    E = [e1 e2 e3];
    E(end+1,:) = [0 0 0];

    n1 = (e1-b1)/norm(e1-b1);
    n2 = (e2-b2)/norm(e2-b2);
    n3 = (e3-b3)/norm(e3-b3);

    movieVector(ind) = getframe(fig1, [30 10 500 400]);
    ind = ind + 1;
end
hold off
% create video file
myWriter = VideoWriter('theta60', 'MPEG-4');
myWriter.FrameRate = 10;
open(myWriter);
writeVideo(myWriter,movieVector);
close(myWriter);

%% for inv condition number plotting
x_pts = linspace(0,length(x_trace),length(x_trace));
for i = 2:length(x_pts)
    plot([x_pts(i) x_pts(i-1)], ...
        [con_nmbrs(i) con_nmbrs(i-1)], '-r', 'LineWidth', 3)
    hold on
end
hold off
title('Inverse Condition Number Evolution: \theta = 60')
xlim([0 6])
xlabel('i-th step in space')
ylabel('inverse condition number')