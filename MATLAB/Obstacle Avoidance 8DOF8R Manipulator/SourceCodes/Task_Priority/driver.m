% this drirver script implements the Task Priority Resolution algorithm
% to path planning of 8-DOF Robotic Arm
clc;
close all;
clear;

%% define constants and variables

% initial and final conditions
qs = (pi/180)*[90 90 -90 90 -90 90 -90 0]';
[p0,frames] = dirkin(qs);
pgoal = [2 2.5]';
delta_q = inf;

% constant
obs = [-0.5 2]';

% tuning parameters
eps = 10^-2; % convergence criterion
radius = 0.3; % radius of influence of obstacle
dt = 1; % time step

% current configuration and position
qcur = qs;
pcur = p0;

% for movieVector
ind = 1;
time = 0;
dist2obstacle = inf;

%% for animation
fig1 = figure(1);


while delta_q > eps
    clf
    % perform direct kinematics
    [pcur, frames] = dirkin(qcur);

    n = length(frames);
    
    % form the top priority task Jacobian (main Jacobian)
    J1 = [cross([0;0;1],frames(1:3,4,n))];
    for i = 1:n-1
        J1(1:3,i+1) = [cross(frames(1:3,3,i),(frames(1:3,4,n)-frames(1:3,4,i)))];
    end
    
    % extract only the translational part of the main Jacobian
    J1 = J1(1:2,:);
    
    % extract the secondary jacobian (2nd task) & the closest point
    [J2,closest2obs] = sense(qcur,obs,frames,radius);

    % draw the robot and plot the expected trajectory line
    drawrobot(qcur)
    hold on
    plot(obs(1),obs(2),'-ok','MarkerSize',8,'MarkerFaceColor','r');
    hold on
    plot(pgoal(1),pgoal(2),'-ok','MarkerSize',10,'MarkerFaceColor','g');
    hold on
    text(0,6,strcat('time =',{' '},num2str(time),' seconds'),"FontSize",10);
    hold on
    text(0,5.4,strcat('\delta_q =',{' '},num2str(delta_q)),"FontSize",10);
    hold on
    text(0,4.9,strcat('distance_o_b_s =',{' '},num2str(dist2obstacle),...
        ' units'),"FontSize",10);
    hold on
    
    % store frames into vector for rendering
    movieVector(ind) = getframe(fig1, [30 10 500 400]);
    
    % compute dq based on solutions in Prob3-Q3
    [dq,delta_q] = RRA_TaskPrio(pcur,pgoal,obs,J1,J2,closest2obs);
    
    % update the current configuration
    qcur = qcur + dq*dt;
    
    % update global values
    ind = ind + 1;
    time = time + dt;
    dist2obstacle = norm(closest2obs-obs);
    
end

% pause the final video frame for t/framerate seconds
t = 30;
for i = 1:t
    clf
    drawrobot(qcur)
    hold on
    plot(obs(1),obs(2),'-ok','MarkerSize',8,'MarkerFaceColor','r');
    hold on
    plot(pgoal(1),pgoal(2),'-ok','MarkerSize',10,'MarkerFaceColor','g');
    hold on
    text(0,6,strcat('time =',{' '},num2str(time),' seconds'),"FontSize",10);
    hold on
    text(0,5.4,strcat('\delta_q =',{' '},num2str(delta_q)),"FontSize",10);
    hold on
    text(0,4.9,strcat('distance_o_b_s =',{' '},num2str(dist2obstacle),...
        ' units'),"FontSize",10);
    hold on
end

hold off
% create video file
myWriter = VideoWriter('tprr-test-7-demo', 'MPEG-4');
myWriter.FrameRate = 10;
open(myWriter);
writeVideo(myWriter,movieVector);
close(myWriter);

