function [q_col, error_col, iter_count_col, response_times, ret] = ...
    RRA(p_goal, a, delta_t, q_init, vmax, vmin, eps_p, lambda_p, ...
    max_iter, view_ang, filename)
%RRA Resolved Rates Algorithm
%
%   Input:
%       p_goal     - goal position
%       a          - vector of link lengths
%       delta_t    - RRA loop step size
%       q_init     - initial configuration
%       vmax       - maximum linear velocity
%       vmin       - minimum linear velocity
%       eps_p      - convergence criterion (radius of convergence)
%       lambda_p   - radius of reducing velocity
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
q_col = [q_init];
error_col = [];
iter_count_col = [];
response_times = [];

% Determine number of goals
k = size(p_goal,1);

% Number of DOF
n = length(a);

% Initialize vector for the current configuration
q_cur = q_init;

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
    delta_p = [];
    
    % ======================FOR ANIMATION ONLY=========================
    % For plotting the home position
    if j==1
        [~, p_cur] = DirKin(a,q_init);
        p_eff_init = p_cur;
    end
    % =================================================================

    while true
        % Initial forward kinematics to  initialize current frames and position
        [frames, p_cur] = DirKin(a, q_cur);
        p_eff(:,end+1) = p_cur;
        delta_p = sqrt((p_cur-p_goal(j, :)')'*(p_cur-p_goal(j, :)'));

        % Test for convergence
        if delta_p <= eps_p
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
            error_col(end+1) = delta_p;
            iter_count_col(end+1) = iter_count;
            response_times(end+1) = toc;
            ret = 0;
            break;
        elseif iter_count >= max_iter
            q_computed = "non-convergent";
            ret = 1;
            break;
        end        

        % Initialize and determine linear velocity magnitude
        vel_mag = 0;
        if delta_p/eps_p > lambda_p
            vel_mag = vmax;
        else
            vel_mag= vmin + ...
            (vmax - vmin)*(delta_p-eps_p)/(eps_p*(lambda_p-1));
        end
        
        % Initialize and determine direction of linear velocity
        n_hat = (p_goal(j, :)'-p_cur)/norm(p_goal(j, :)'-p_cur);
        
        % Calculate the linear velocity
        p_dot = vel_mag*n_hat;
        
        % Form the Jacobian
        J = [cross([0;0;1],p_cur)];
        for i = 1:n-1
            J(1:3,i+1) = [cross(frames(1:3,3,i),(p_cur-frames(1:3,4,i)))];  
        end
        
        % Compute rate of change of joint variables
        q_dot = pinv(J)*p_dot;
        
        % Compute new values of q
        q_cur = q_cur + q_dot*delta_t;

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
    
        % Increment iteration count
        iter_count = iter_count+1;
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
title(plt, 'Resolved Rates Algorithm Simulation')

axes1 = nexttile;
copyobj(last_frame, axes1);
title('Isometric View')
view(45, 45)
grid on
grid minor
xlabel('x')
ylabel('y')
zlabel('z')

axes2 = nexttile;
copyobj(last_frame, axes2);
title('Side View 1')
view(0, 0)
grid on
grid minor
xlabel('x')
ylabel('y')
zlabel('z')

axes3 = nexttile;
copyobj(last_frame, axes3);
title('Side View 2')
view(90, 0)
grid on
grid minor
xlabel('x')
ylabel('y')
zlabel('z')

axes4 = nexttile;
copyobj(last_frame, axes4);
title('Top View')
view(0, 90)
grid on
grid minor
xlabel('x')
ylabel('y')
zlabel('z')

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




