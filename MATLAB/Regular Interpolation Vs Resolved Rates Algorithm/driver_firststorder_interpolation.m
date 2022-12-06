% This driver script runs the DirKin function for PUMA560.
% This is for problem 1 task 3.
clc;
close all;
clear all;

% configuration boundaries
qstart = (pi/180)*[0 90 0 0 0 0]';
qgoal = (pi/180)*[-45 60 45 45 30 45]';

% time boundaries
t0 = 0;
tf = 5;
step_count = 500;
stepsize = (tf-t0)/step_count;

% define time vector
t = linspace(t0, tf, step_count);

% interpolate rate of change of all config angles
dq1 = (qgoal(1)-qstart(1))/step_count;
dq2 = ((qgoal(2)-qstart(2))/(qgoal(1)-qstart(1)))*dq1;
dq3 = ((qgoal(3)-qstart(3))/(qgoal(1)-qstart(1)))*dq1;
dq4 = ((qgoal(4)-qstart(4))/(qgoal(1)-qstart(1)))*dq1;
dq5 = ((qgoal(5)-qstart(5))/(qgoal(1)-qstart(1)))*dq1;
dq6 = ((qgoal(6)-qstart(6))/(qgoal(1)-qstart(1)))*dq1;

% current config
qcur = qstart;

% tooltip position in space per time step (for plotting)
pvec = [];

% for movieVector index only
ind = 1;

% figure handle
fig = figure;

for i = t0:stepsize:tf
    %clear figure
    clf
    
    % perform direct kinematics
    [frames, p, R] = DirKin(qcur);
    
    % plot trajectory of tooltip
    pvec = [pvec p];
    [row,col] = size(pvec);
    if i ~= t0
        for j = 1:col-1
            %plot trajectory of tooltip
            plot3([pvec(1,j) pvec(1,j+1)],[pvec(2,j) pvec(2,j+1)],[pvec(3,j) pvec(3,j+1)], '-m', 'LineWidth', 3)
            hold on
        end
    end
    
    view(45,30)
    axis([-1.5,1.5,-1.5,1.5,0,1.8]);
    grid on; 
    
    % plot 3D robotic arm
    draw_puma560(frames,'surface')
    
    % show current time
    text(-0.5,1,1.7,strcat('Time = ', num2str(i), ' seconds'),'FontWeight','bold', 'FontSize', 20);
    
    % plot legend for tooltip trajectory
    lgnd = plot3(0,0,0,'-m');
    legend(lgnd,'tool tip trajectory','Location','northeast')
    hold on
    hold on
    
    % update qcur values
    q1 = qcur(1) + dq1;
    q2 = qcur(2) + dq2;
    q3 = qcur(3) + dq3;
    q4 = qcur(4) + dq4;
    q5 = qcur(5) + dq5;
    q6 = qcur(6) + dq6;
    qcur = [q1 q2 q3 q4 q5 q6];
  
    % store frames into vector for rendering
    movieVector(ind) = getframe(fig, [30 20 500 400]);
    ind = ind + 1;
end

% create video file
myWriter = VideoWriter('puma560_1storderinterpolation', 'MPEG-4');
myWriter.FrameRate = 10;
open(myWriter);
writeVideo(myWriter,movieVector);
close(myWriter);