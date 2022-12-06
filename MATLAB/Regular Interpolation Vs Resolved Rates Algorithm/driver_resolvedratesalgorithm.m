% This driver script runs the DirKin and RRA function for PUMA560.
% This is for problem 1 task 4.
clc;
close all;
clear all;

% intial config
qstart = (pi/180)*[0 90 0 0 0 0]';

% convergence crteria
eps_p = 0.001;
eps_o = 0.0524;

% goal parameters
pd = [0.5054; -0.6297; 1.1176];
nx = -0.4434;
ny = 0.6648;
nz = -0.6013;
theta = 138.59*pi/180;
Rd = vrrotvec2mat([nx ny nz theta]);

% time parameters
t0 = 0;
dt = 0.01;

% current config
qcur = qstart;

% tooltip position in space per time step (for plotting)
pvec = [];

% for movieVector index only
ind = 1;

% figure handle
fig = figure;

% for timer
i = t0;

while true
    clf    
    
    % perform direct kinematics
    [frames, p, R] = DirKin(qcur);
    
    % plot trajectory of tooltip
    pvec = [pvec p];
    [row,col] = size(pvec);
    if i ~= t0
        for j = 1:col-1
            %plot trajectory of tooltip
            plot3([pvec(1,j) pvec(1,j+1)],[pvec(2,j) pvec(2,j+1)],...
                [pvec(3,j) pvec(3,j+1)], '-m', 'LineWidth', 3)
            hold on
        end
    end
    
    view(45,30)
    axis([-1.5,1.5,-1.5,1.5,0,1.8]);
    grid on; 
    
    % plot 3D robotic arm
    draw_puma560(frames,'surface')
    
    % show current time
    text(-1,1,1.7,strcat('Time = ', num2str(i), ' seconds'),...
        'FontWeight','bold', 'FontSize', 10);
    
    % run resolved rates algorithm fcn
    [dq, delta_p, delta_o] = RRA(frames,p,pd,Rd);
    
    % show current error norms
    text(-1,1,1.5,strcat('Position Error Norm (delta_p)= ', ...
        num2str(delta_p)),'FontWeight','bold', 'FontSize', 10);
    text(-1,1,1.3,strcat('Orientation Error Norm (delta_o)= ', ...
        num2str(delta_o)),'FontWeight','bold', 'FontSize', 10);
   
    % plot legend for tooltip trajectory
    lgnd = plot3(0,0,0,'-m');
    legend(lgnd,'tool tip trajectory','Location','northeast')
    hold on
    
    % convergence check
    if (delta_p <= eps_p) || (delta_o <= eps_o)
        break;
    end
    
    % update current config values
    qcur = qcur + dq*dt;
    
    % store frames into vector for rendering
    movieVector(ind) = getframe(fig, [30 20 500 400]);
    
    % increment movieVector and timer variables
    ind = ind + 1;
    i = i + dt;
end

% create video file
myWriter = VideoWriter('puma560_resolvedrates', 'MPEG-4');
myWriter.FrameRate = 10;
open(myWriter);
writeVideo(myWriter,movieVector);
close(myWriter);
