clear;
clc;
close all;

%% define constants, vectors and functions

a = 0.5; % for r
thetas = linspace(0,3600,360); % gripper theta steps
eps_p = 0.001; % convergence criterion for resolved rate algorrithm (RRA)

q0 = [0 ; pi/4; pi/4; pi/4]; % starting configuration
[frames, x_gripper] = dirkin(q0); % get starting position
x0 = x_gripper(1); % starting x position
y0 = x_gripper(2); % starting y position

% lambda functions for dynamic computations of next destination point
r = @(theta) a*cos(3*theta);
x = @(r_cur,theta) r_cur*cos(theta) + x0;
y = @(r_cur,theta) r_cur*sin(theta) + y0;

qcur = q0; % current configuration
pcur = [x0;y0]; % current position
framescur = frames; % current frames
delta_p = inf; % set delta_p initially to infinity for RRA convergence
qvals = []; % all configuration values as theta varies
pvals = [pcur]; % all gripper position coordinates as theta varies

%{
% for movieVector index only
ind = 1; % for video rendering only
%}

%% animation from home position to start of theta trajectory

fig=figure(1);
th = thetas(1)*pi/180; % convert to radians
r1 = r(th); 
pd = [x(r1,th);y(r1,th)]; % comnpute next desired point
while true
    clf
    [qdot,delta_p] = RRA(qcur,pcur,pd);
    drawrobot(qcur) % draw robot config after each RRA loop
    hold on
    
    % plot the gripper trjectory
    [row, col] = size(pvals);
    if col > 1
        plt = line(pvals(1,:),pvals(2,:), 'Color', 'm');
        legend(plt, 'Gripper Trajectory');
    end
    
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

%% animation from theta 0 to 3600 degrees

for i = 1:length(thetas)
    clf
    
    th = thetas(i)*pi/180; % convert to radians
    rcur = r(th);
    pd = [x(rcur,th);y(rcur,th)]; % comnpute next desired point
    [qdot,delta_p] = RRA(qcur,pcur,pd);
    
    % RRA loop
    while delta_p > eps_p
        
        qcur = qcur + qdot*0.001;
        [framescur,pcur] = dirkin(qcur);
        [qdot,delta_p] = RRA(qcur,pcur,pd);
        
    end
    
    drawrobot(qcur) % draw robot config after each RRA loop
    hold on
    
    qvals = [qvals qcur]; % update qvals for section 3 plotting 
    pvals = [pvals pcur]; % update pvals for trajectory plotting
    delta_p = inf; % set delta_p back to inf for the next theta loop
    
    % plot the gripper trjectory
    [row, col] = size(pvals);
    if col > 1
        plt = line(pvals(1,:),pvals(2,:), 'Color', 'm');
        legend(plt, 'Gripper Trajectory');
    end 
    
    getframe; % comment out if rendering video
    %{
    % for video rendering only
    movieVector(ind) = getframe(fig, [30 20 500 400]);   
    ind = ind + 1;
    %}
    
end

%{
% create video file
myWriter = VideoWriter('leastnorm-pinverse', 'MPEG-4');
myWriter.FrameRate = 30;
open(myWriter);
writeVideo(myWriter,movieVector);
close(myWriter);
%}

%% for plotting of Joint Values VS Theta

fig2 = figure(2);
thetas = thetas*pi/180;
for i = 1:length(thetas)-1
    plot([thetas(i) thetas(i+1)],[qvals(1,i) qvals(1,i+1)],'-r','LineWidth',3)
    hold on
    plot([thetas(i) thetas(i+1)],[qvals(2,i) qvals(2,i+1)],'-g','LineWidth',3)
    hold on
    plot([thetas(i) thetas(i+1)],[qvals(3,i) qvals(3,i+1)],'-b','LineWidth',3)
    hold on
    plot([thetas(i) thetas(i+1)],[qvals(4,i) qvals(4,i+1)],'-k','LineWidth',3)
    hold on
    if i == length(thetas)-1
        leg1 = plot(0,0,'r');
        leg2 = plot(0,0,'g');
        leg3 = plot(0,0,'b');
        leg4 = plot(0,0,'k');
        legends = [leg1 leg2 leg3 leg4];
    end
    hold on
    
end

title('Joint Values VS \Theta')
grid on
grid minor
xlabel('\Theta (radians)')
ylabel('Joint Values - q_1(m) & q_2, q_3, q_4(rad)')
legend(legends,'q_1 (meters)', 'q_2 (radians)', 'q_3 (radians)', 'q_4 (radians)')
xlim([0 thetas(end)])

% dynamic theta labeling per pi/2 radians
dx = pi/2;
xticks([0:dx:thetas(end)]);
x_labels = {};
for i = 0:thetas(end)/dx
    x_labels{i+1} = strcat(num2str(i/2),'\pi');
end
xticklabels(x_labels);

fprintf('Solution (Q1 Task 3) is not repeatable. \nThe joint values continue to deviate away from their respective initial values.\n')
