clear;
clc;
close all;

%% define constants, vectors and functions

thetas = linspace(210,300,90); % gripper theta steps
eps_p = 0.001; % convergence criterion for resolved rate algorrithm (RRA)

q0 = [0 ; pi/4; pi/4; pi/4]; % starting configuration
[frames, x_gripper] = dirkin(q0); % get starting position
x0 = x_gripper(1); % starting x position
y0 = x_gripper(2); % starting y position

% circle center
xc = -1.0;
yc = 0.5;

% lambda functions for dynamic computations of next destination point
r = 0.5;
x = @(theta) r*cos(theta) + xc;
y = @(theta) r*sin(theta) + yc;

qcur = q0; % current configuration
pcur = [x0;y0]; % current position
framescur = frames; % current frames
delta_p = inf; % set delta_p initially to infinity for RRA convergence
pvals = [pcur]; % all gripper position coordinates as theta varies
alphas = []; % evolution of alphas wrt to theta

%{
% for movieVector index only
ind = 1; % for video rendering only
%}
%% animation from home position to start of theta trajectory

fig=figure(1);
th = thetas(1)*pi/180; % convert to radians
pd = [x(th);y(th)]; % comnpute next desired point
while true
    clf
    [qdot,delta_p,alpha] = RRA(qcur,[xc;yc],pcur,pd);
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

alphas = [alphas alpha];


%% animation for theta trajectory
for i = 2:length(thetas)
    clf
    
    th = thetas(i)*pi/180; % convert to radians
    pd = [x(th);y(th)]; % comnpute next desired point
    [qdot,delta_p,alpha] = RRA(qcur,[xc;yc],pcur,pd);
    
    % RRA loop
    while delta_p > eps_p
        
        qcur = qcur + qdot*0.001;
        [framescur,pcur] = dirkin(qcur);
        [qdot,delta_p,alpha] = RRA(qcur,[xc;yc],pcur,pd);
        
        
    end
    
    drawrobot(qcur) % draw robot config after each RRA loop
    hold on
    
    alphas = [alphas alpha]; % update alphas for section 3 plotting
    pvals = [pvals pcur]; % update pvals for trajectory plotting
    delta_p = inf; % set delta_p back to inf for the next theta loop
    
    
    % plot the gripper trjectory
    [row, col] = size(pvals);
    if col > 1
        plt = line(pvals(1,:),pvals(2,:), 'Color', 'm');
        legend(plt, 'Gripper Trajectory')
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
myWriter = VideoWriter('leastnorm-pinverse', 'MPEG-4');
myWriter.FrameRate = 30;
open(myWriter);
writeVideo(myWriter,movieVector);
close(myWriter);
%}

%% for plotting of Alpha VS Theta
fig2 = figure(2);
thetas = thetas*pi/180;
for i = 1:length(thetas)-1
    plot([thetas(i) thetas(i+1)],[alphas(i) alphas(i+1)],'-r','LineWidth',3)
    hold on
    if i == length(thetas)-1
        leg1 = plot(0,0,'r');
        legends = [leg1];
    end
    hold on
    
end

title('\alpha VS \Theta')
grid on
grid minor
xlabel('\Theta (radians)')
ylabel('\alpha (radians)')
legend(legends,'\alpha (radians)')
xlim([thetas(1) thetas(end)])

% dynamic theta labeling per pi/2 radians
dx = 10*pi/180;
xticks([thetas(1):dx:thetas(end)]);
x_labels = {};
for i = 0:(thetas(end)-thetas(1))/dx
    if i == 0
        x_labels{i+1} = strcat(num2str(thetas(1)/pi),'\pi');
    else
        x_labels{i+1} = strcat(num2str(thetas(i*10)/pi),'\pi');
    end
end
xticklabels(x_labels);
