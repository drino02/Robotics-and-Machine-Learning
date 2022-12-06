function [q_col, error_col, iter_count_col, response_times, ret] = ...
    InvKin(p_goal, a, gamma, step_size, guess_init, max_iter, view_ang, filename)
%INVKIN Pure Inverse Kinematics via Newton-Rhapson Method
%
%   Input:
%       p_goal     - goal position
%       a          - vector of link lengths
%       gamma      - convergence criterion
%       step_size  - PIK loop step size
%       guess      - column vector of the guess for the initial configuration
%       max_iter   - maximum iteration count for which beyond, 
%                    the solution is considered non-convergent
%       view_ang       - view angles for animation [azimuth, elevation]
%       filename   - filename of output animation file
%   Output:
%       q_computed - converged solution of the goal joint configuration
%       error      - absolute error between the goal position and the
%                   converged solution
%       iter_count - iteration count until convergence

% Initialize output containers
q_col = [guess_init];
error_col = [];
iter_count_col = [];
response_times = [];

% Determine number of goals
k = size(p_goal,1);

% Number of DOF
n = length(a);

% Initialize vector for the current configuration
q_cur = guess_init;

% ======================FOR ANIMATION ONLY=========================
ind = 1;  % movie vector frame index
fig = figure;  % figure container
p_eff_init =[];  % for plotting the home position
p_eff = [];  % for plotting the trajectory of the end-effector
% =================================================================

for j = 1:k
    tic
    % Initialize iteration count and error
    iter_count = 0;
    error = 100;
    
    % Initial forward kinematics to  initialize current frames and position
    [frames, p_cur] = DirKin(a, q_cur);
    p_eff(:,end+1) = p_cur;
    
    % ======================FOR ANIMATION ONLY=========================
    % For plotting the home position
    if j==1
        p_eff_init = p_cur;
    end
    % =================================================================

    while true
        
        % Test for convergence
        if error <= gamma
            % COLLISION AVOIDER/ROTATION OPTIMIZER: 
            % if absolute value of the angle is greater than 180deg, 
            % go the opposite direction
            for i = 1:length(q_cur)
                if q_cur(i) < -180
                    q_cur(i) = q_cur(i) + 360;
                elseif q_cur(i) > 180
                    q_cur(i) = q_cur(i) - 360;
                end
            end
            q_col(:,end+1) = q_cur;
            error_col(end+1) = error;
            iter_count_col(end+1) = iter_count;
            response_times(end+1) = toc;
            ret = 0;
            break;
        elseif iter_count >= max_iter
            q_computed = "non-convergent";
            ret = 1;
            break;
        end
    
        % Form the Jacobian
        J = [cross([0;0;1],p_cur)];
        for i = 1:n-1
            J(1:3,i+1) = [cross(frames(1:3,3,i),(p_cur-frames(1:3,4,i)))];  
        end
        
        % Implement numerical method
        q_cur = q_cur - step_size*pinv(J)*(p_cur-p_goal(j,:)');

        % Determine new frames and position
        [frames, p_cur] = DirKin(a, q_cur);     


        % ======================FOR ANIMATION ONLY=========================
        % Plot home position, goal points and end-effector trajectory
        plot3(p_eff_init(1), p_eff_init(2), p_eff_init(3), 'ok', ...
            'MarkerSize', 15, 'MarkerFaceColor', 'y')
        hold on
        scatter3(p_goal(:,1), p_goal(:,2), p_goal(:,3), 150, 'green', ...
            'filled');
        hold on
        plot3(p_eff(1,:), p_eff(2,:), p_eff(3,:), '*r');
        hold on

        % For the figure legend
        leg1 = plot3(p_eff_init(1), p_eff_init(2), p_eff_init(3), 'ok', ...
            'MarkerFaceColor', 'y', DisplayName='Home position');
        hold on
        leg2 = plot3(p_goal(1,1), p_goal(1,2), p_goal(1,3), 'og', ...
            'MarkerFaceColor', 'g', DisplayName='Goal positions');
        hold on
        leg3 = plot3(p_eff(1,1), p_eff(2,1), p_eff(3,1), '*r', ...
            DisplayName='End-effector trajectory points');
        hold on

        p_eff(:,end+1) = p_cur;  % current end-effector position
        DrawRobot(frames,view_ang)   % robot plotter
        legend([leg1, leg2, leg3], 'Location','northwest')  % draw legend
        movieVector(ind) = getframe(fig, [30 10 500 400]);  % figure frame
        ind = ind+1;   % increment movieVector index
        % =================================================================

        % Compute the error
        error = norm(p_cur-p_goal(j,:)');
    
        % Increment iteration count
        iter_count = iter_count+1;
        
        % NORMALIZER: if joint angles are more than 360degrees, bring there
        % values down within the range of one revolution
        for i = 1:length(q_cur)
            if abs(q_cur(i)) > 360
                q_cur(i) = q_cur(i)/360;
            end
        end
    end

    if ret ~= 0
        disp(['One of the goals cannot be reached. ' ...
            'Algorithm did not converge.']);
        break;
    end

end

% ====================== FOR PLOT ONLY=========================
last_frame = gca().Children;

figure(2)
plt = tiledlayout(2, 2);
title(plt, 'Pure Inverse Kinematics Simulation')

axes1 = nexttile;
copyobj(last_frame, axes1);
title('Isometric View')
view(45, 5)
grid on
grid minor
xlabel('x')
ylabel('y')
zlabel('z')
xlim([-1 1])
ylim([-1 1])
zlim([0 2])

axes2 = nexttile;
copyobj(last_frame, axes2);
title('Side View 1')
view(0, 0)
grid on
grid minor
xlabel('x')
ylabel('y')
zlabel('z')
xlim([-1 1])
ylim([-1 1])
zlim([0 2])

axes3 = nexttile;
copyobj(last_frame, axes3);
title('Side View 2')
view(90, 0)
grid on
grid minor
xlabel('x')
ylabel('y')
zlabel('z')
xlim([-1 1])
ylim([-1 1])
zlim([0 2])

axes4 = nexttile;
copyobj(last_frame, axes4);
title('Top View')
view(0, 90)
grid on
grid minor
xlabel('x')
ylabel('y')
zlabel('z')
xlim([-1 1])
ylim([-1 1])
zlim([0 2])

saveas(gcf, strcat(filename, '-plot.png'));
% =================================================================


% ======================FOR ANIMATION ONLY=========================
% create video file
myWriter = VideoWriter(filename, 'MPEG-4');
myWriter.FrameRate = 60;
open(myWriter);
writeVideo(myWriter,movieVector);
close(myWriter);
% =================================================================




