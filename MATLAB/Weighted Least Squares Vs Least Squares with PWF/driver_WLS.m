% Weighted Least Squares (WLS) Solution
clear;
clc;
close all;

%% define constants, vectors and functions

time = linspace(0,1,100);
eps_p = 0.001; % convergence criterion for resolved rate algorrithm (RRA)

q0 = [0; 0; pi/4; pi/4; pi/4]; % assumed starting configuration
[frames, x_gripper] = dirkin(q0); % get starting position
x0 = x_gripper(1); % starting x position
y0 = x_gripper(2); % starting y position


% X and Y values
x = @(t_cur) -5*sin(2*pi*t_cur/5);
y = 0;

qcur = q0; % current configuration
pcur = [x0;y0]; % current position
delta_p = inf; % set delta_p initially to infinity for RRA convergence
qvals = [];
pvals = []; % all gripper position coordinates as theta varies

%{
% for movieVector index only
ind = 1; % for video rendering only
%}

%% animation from home position to start of time trajectory

fig=figure(1);
td = 0; % initial t
pd = [x(td);y]; % comnpute next desired point
while true
    clf
    [qdot,delta_p] = RRA_WLS(qcur,pcur,pd);
    drawrobot(qcur) % draw robot config after each RRA loop
    hold on
    
    % plot the gripper trjectory
    [row, col] = size(pvals);
    if col > 1
        plt = line(pvals(1,:),pvals(2,:), 'Color', 'm');
        legend(plt, 'Gripper Trajectory');
    end
    
    hold on
    
    getframe; % comment out if rendering video
    %{
    % for video rendering only
    movieVector(ind) = getframe(fig, [30 20 500 400]);
    ind = ind + 1;
    %}
    
    % Test for convergence
    if delta_p <= eps_p
        break;
    end
    
    qcur = qcur + qdot*0.1;
    [framescur,pcur] = dirkin(qcur);
    
    pvals = [pvals pcur]; % update pvals for trajectory plotting
    
end

%% animation for time trajectory
for t = 0:0.01:1
    clf
    
    pd = [x(t);y]; % compute next desired point
    [qdot,delta_p] = RRA_MinNorm(qcur,pcur,pd);
    
    % RRA loop
    while delta_p > eps_p
        qcur = qcur + qdot*0.001;
        [framescur,pcur] = dirkin(qcur);
        [qdot,delta_p] = RRA_MinNorm(qcur,pcur,pd);
    end
    
    drawrobot(qcur) % draw robot config after each RRA loop
    hold on
    
    qvals = [qvals qcur]; % update qvals for jointpath vs time plot
    pvals = [pvals pcur]; % update pvals for trajectory plotting
    delta_p = inf; % set delta_p back to inf for the next theta loop
    
    % plot the gripper trajectory
    [row, col] = size(pvals);
    if col > 1
        plt = line(pvals(1,:),pvals(2,:), 'Color', 'm');
        legend(plt, 'Gripper Trajectory');
    end
    
    hold on
    
    getframe; % comment out if rendering video
    %{
    % for video rendering only
    movieVector(ind) = getframe(fig, [30 20 500 400]);
    ind = ind + 1;
    %}
    
end

%{
% create video file
myWriter = VideoWriter('WLS', 'MPEG-4');
myWriter.FrameRate = 30;
open(myWriter);
writeVideo(myWriter,movieVector);
close(myWriter);
%}


%% for plotting of Joint paths VS Time
fig2 = figure(2);
for i = 1:length(time)-1
    plot([time(i) time(i+1)],[qvals(1,i) qvals(1,i+1)],'-r','LineWidth',3)
    hold on
    plot([time(i) time(i+1)],[qvals(2,i) qvals(2,i+1)],'-g','LineWidth',3)
    hold on
    plot([time(i) time(i+1)],[qvals(3,i) qvals(3,i+1)],'-b','LineWidth',3)
    hold on
    plot([time(i) time(i+1)],[qvals(4,i) qvals(4,i+1)],'-k','LineWidth',3)
    hold on
    plot([time(i) time(i+1)],[qvals(5,i) qvals(5,i+1)],'-m','LineWidth',3)
    hold on
    if i == length(time)-1
        leg1 = plot(0,0,'r');
        leg2 = plot(0,0,'g');
        leg3 = plot(0,0,'b');
        leg4 = plot(0,0,'k');
        leg5 = plot(0,0,'m');
        legends = [leg1 leg2 leg3 leg4 leg5];
    end
    hold on
    
end

title('Joint Values VS Time')
grid on
grid minor
xlabel('Time (seconds)')
ylabel('Joint Value (radians)')
legend(legends,'q_1', 'q_2', 'q_3', 'q_4', 'q_5')


