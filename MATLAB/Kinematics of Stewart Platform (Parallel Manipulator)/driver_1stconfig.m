clc;
close all;
clear all;

% constants
r_p = 100; % in mm
r_b = 200; % in mm
alpha_p = 10; % in degrees
alpha_b = 10; % in degrees
eps = 0.1; % in mm
dt = 0.1; % arbitrary time step
theta_dot = -pi/180*dt; % arbitrary rate of rotation about axis omega

% goal
q_h = [213.8014 234.6068 261.0721 210.6947 248.6608 214.0245]';

% home config (initial values)
t_0 = [0 0 180]'; % in mm
R_0 = diag([1 1 1]);

% current values
t_cur = t_0;
R_cur = R_0;
delta_q = inf; % initilize convergence criterion to inf

% base vertices, bi
b1 = [r_b*cosd(50) r_b*sind(50) 0]';
b2 = [r_b*cosd(70) r_b*sind(70) 0]';
b3 = [r_b*cosd(170) r_b*sind(170) 0]';
b4 = [r_b*cosd(190) r_b*sind(190) 0]';
b5 = [r_b*cosd(290) r_b*sind(290) 0]';
b6 = [r_b*cosd(310) r_b*sind(310) 0]';

% for movieVector
ind = 1;

fig1 = figure(1);

while true
    clf
    
    % values of pi with respect to the center of the platform (t)
    a1 = [r_p*cosd(10) r_p*sind(10) 0]';
    a2 = [r_p*cosd(110) r_p*sind(110) 0]';
    a3 = [r_p*cosd(130) r_p*sind(130) 0]';
    a4 = [r_p*cosd(230) r_p*sind(230) 0]';
    a5 = [r_p*cosd(250) r_p*sind(250) 0]';
    a6 = [r_p*cosd(-10) r_p*sind(-10) 0]';
    
    % values of pi with respect to the world frame
    p1 = t_cur + R_cur*a1;
    p2 = t_cur + R_cur*a2;
    p3 = t_cur + R_cur*a3;
    p4 = t_cur + R_cur*a4;
    p5 = t_cur + R_cur*a5;
    p6 = t_cur + R_cur*a6;
    
    % vectors respresenting the length of each respective link
    s1 = p1 - b1;
    s2 = p2 - b2;
    s3 = p3 - b3;
    s4 = p4 - b4;
    s5 = p5 - b5;
    s6 = p6 - b6;
    
    
    % current link lengths qi
    q_cur = [norm(s1) norm(s2) norm(s3) norm(s4) norm(s5) norm(s6)]';
    
    delta_q =  norm(q_h-q_cur);
    
    % plot
    R_plot = rotm2axang(R_cur);
    A_wcs = [p1 p2 p3 p4 p5 p6];
    B = [b1 b2 b3 b4 b5 b6];
    Drawrobot_sg(A_wcs,B);
    text(0,0,420,strcat('\delta_q =', {' '}, ...
        num2str(delta_q)),'FontWeight','bold', 'FontSize', 10);
    text(0,0,380,strcat('t_c_u_r =', {' '}, '[', ...
        num2str(t_cur(1)), {' '}, num2str(t_cur(2)), {' '},...
        num2str(t_cur(3)), ']'),'FontWeight','bold', 'FontSize', 10);
    text(0,0,340,strcat('R_c_u_r =', {' '}, 'rotd([', ...
        num2str(R_plot(1)), {' '}, num2str(R_plot(2)), {' '},...
        num2str(R_plot(3)), '], ',num2str(R_plot(4)*180/pi),')'),...
        'FontWeight','bold', 'FontSize', 10);
    xlim([-250 200]);
    ylim([-200 250]);
    zlim([0 450]);
    view(20,30)
    movieVector(ind) = getframe(fig1, [30 10 500 400]);
    ind = ind + 1;
    
    % convergence criterion
    if delta_q < eps
        break;
    end

    % Jacobian
    J = [s1' cross(s1',R_cur*a1); ...
    s2' cross(s2',R_cur*a2); ...
    s3' cross(s3',R_cur*a3); ...
    s4' cross(s4',R_cur*a4); ...
    s5' cross(s5',R_cur*a5); ...
    s6' cross(s6',R_cur*a6)];
    
    % modified rates
    qdot_min = 10;
    qdot_max = 1000;
    lambda_q = 10;

    nhat = (q_h - q_cur)/norm(q_h - q_cur);

    if  delta_q/eps >  lambda_q
        qdot_mag = qdot_max;
    else
        qdot_mag = qdot_min + ...
            ((qdot_max-qdot_min)*(delta_q-eps))*0.01/(eps*(lambda_q-1));
    end
    
    q_dot = qdot_mag*nhat;
    x_dot = pinv(J)*q_dot;

    omega = x_dot(4:6)';

    % rotation correction
    R_corr = axang2rotm([omega/norm(omega) theta_dot]); 
    
    % update platform position and orientation
    R_cur = R_corr*R_cur;
    t_cur = t_cur + x_dot(1:3,1)*dt;

end

hold off
% create video file
myWriter = VideoWriter('firstconfig', 'MPEG-4');
myWriter.FrameRate = 10;
open(myWriter);
writeVideo(myWriter,movieVector);
close(myWriter);
    
    
    

