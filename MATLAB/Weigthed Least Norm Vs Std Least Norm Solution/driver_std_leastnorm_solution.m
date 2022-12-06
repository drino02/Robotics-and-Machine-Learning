% this drirver sscript runs the dirkin and drawrobot functions, plots the
% trajectory of the robotic arm, and creates an animation. (HW2 Prob3-Q5-3)
clc;
close all;
clear all;

%% define constants and variables

% starting and goal parameters
qs = [45 -60 -70 -60]';
[ps,frames] = dirkin(qs);
pgoal = -ps;

% gripper speed
vlin = 2;

% distance from starting point to goal
dist2goal = norm(pgoal-ps);

% normalized change in velocity in task space
vdirect = atan2((pgoal(2)-ps(2)),(pgoal(1)-ps(1)));
d_xdes = vdirect*vlin*[cos(vdirect); sin(vdirect)];
d_xdes = (d_xdes./norm(d_xdes));



% dimensionless parameter, t
t0 = 0;
tf = 1;
dt = 0.2;
step = 0.01;

% current configuration (as change progressess wrt to t)
qcur = qs;

% for movieVector index only
ind = 1;

% for joint speed vs t trajectory plotting
jt_sp =[];
t_plot = [];
config = [];

%% for animation
fig1 = figure(1);
dist = 0;

while dist <= dist2goal
    clf
    % perform direct kinematics
    [x, frames] = dirkin(qcur);
    
    n = length(frames);
    
    % form the Jacobian
    J = [cross([0;0;1],frames(1:3,4,n)); [0; 0; 1]];
    for i = 1:n-1
        J(1:6,i+1) = [cross(frames(1:3,3,i),(frames(1:3,4,n)-frames(1:3,4,i)));...
            frames(1:3,3,i)];
    end
    
    % extract only the translational part of the Jacobian
    J = J(1:2,1:end);
    
    % draw the robot and plot the expected trajectory line
    drawrobot(qcur)
    hold on
    plot([pgoal(1) ps(1)], [pgoal(2) ps(2)], '-or','MarkerSize', 3);
    hold on
    text(ps(1)+0.1,ps(2)-0.1,'t=0','Color','r')
    hold on
    text(pgoal(1)-0.175,pgoal(2)-0.1,'t=1','Color','r')
    hold on
    title('Trajectory form t=0 to t=1')
    hold on
    
    % store frames into vector for rendering
    movieVector(ind) = getframe(fig1, [30 10 500 400]);
    
    % compute dq based on solutions in Prob3-Q3
    dq = pinv(J)*d_xdes;
    
    % update plotting vectors
    jt_sp = [jt_sp dq];
    t_plot = [t_plot tf*norm(x-ps)/dist2goal];
    config = [config qcur];
    
    % update the current configuration
    qcur = dq + qcur;
    
    % update global values
    dist = norm(x-ps);
    ind = ind + 1;
    
end
hold off
% create video file
myWriter = VideoWriter('std-leastnorm.mp4', 'MPEG-4');
myWriter.FrameRate = 3;
open(myWriter);
writeVideo(myWriter,movieVector);
close(myWriter);

%% for plotting of joint speed vs t
fig2 = figure(2);
for i = 1:length(t_plot)-1
    plot([t_plot(i) t_plot(i+1)],[jt_sp(1,i) jt_sp(1,i+1)],'-r','LineWidth',3)
    hold on
    plot([t_plot(i) t_plot(i+1)],[jt_sp(2,i) jt_sp(2,i+1)],'-g','LineWidth',3)
    hold on
    plot([t_plot(i) t_plot(i+1)],[jt_sp(3,i) jt_sp(3,i+1)],'-b','LineWidth',3)
    hold on
    plot([t_plot(i) t_plot(i+1)],[jt_sp(4,i) jt_sp(4,i+1)],'-k','LineWidth',3)
    hold on
    if i == length(t_plot)-1
        leg1 = plot(0,0,'r');
        leg2 = plot(0,0,'g');
        leg3 = plot(0,0,'b');
        leg4 = plot(0,0,'k');
        legends = [leg1 leg2 leg3 leg4];
    end
    hold on
    
end

title('Joint Speed VS t')
grid on
grid minor
xlabel('t')
ylabel('Joint Speed (radians/sec)')
legend(legends,'theta 1 speed', 'theta 2 speed', 'theta 3 speed', 'theta 4 speed')
xlim([0 1])

%% for still plot of robot arm
fig3 = figure(3);

t_stops = [0 0.2 0.4 0.6 0.8 1];
t_plot = round(t_plot,2);

for i = 1:length(t_stops)
    ind = find(t_plot == t_stops(i));
    drawrobot(config(:,ind));
    hold on
end
plot([pgoal(1) ps(1)], [pgoal(2) ps(2)], '-or','MarkerSize', 3);
hold on
text(ps(1)+0.1,ps(2)-0.1,'t=0','Color','r')
hold on
text(pgoal(1)-0.175,pgoal(2)-0.1,'t=1','Color','r')
hold on
title('Robot Pose at t=[0,0.2,0.4,0.6,0.8,1]')
hold off
grid on
grid minor