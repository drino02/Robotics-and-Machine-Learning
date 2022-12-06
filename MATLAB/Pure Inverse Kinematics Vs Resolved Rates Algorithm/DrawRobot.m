function DrawRobot(frames, view_angles)
%DRAWROBOT
%
%   Input:
%       frames - n-dmensional matrix array containing all homogeneous
%                transformation matrices from first joint to the last
%       view_angles  - 2-element vector containing azimuth and elevation angles
%   Output:
%       plot.

n = size(frames, 3);                % number of joints
link_girth = [12.5, 10, 8, 6, 4];   % girth of each link
joint_size = 10;                    % size of the inner joint marker
joint_outline = 20;                 % size of the outer joint marker

% Initialize cvontainers for joint positions with the origin
p = [0;0;0];

for i = 2:n+1
    p(:,i) = frames(1:3,4,i-1);
    if i < n+1
        % Plot layout for all links except the last
        plot3([p(1,i-1) p(1,i)], [p(2,i-1) p(2,i)], [p(3,i-1) p(3,i)], ...
            '-*', 'Color', '#3B3A3A', 'LineWidth', link_girth(i-1), ...
            'MarkerSize', joint_size, 'MarkerFaceColor', ...
            'blue', 'MarkerEdgeColor', 'blue')
        hold on
        
        % Plot the first joint until the second to the last one
        plot3([p(1,i-1) p(1,i)], [p(2,i-1) p(2,i)], [p(3,i-1) p(3,i)], ...
            'ok', 'LineWidth', 3, 'MarkerSize', joint_outline, ...
            'MarkerEdgeColor', '#595655')
        hold on
    else
        % Plot different layout for the last link 
        % to accentuate the end-effector
        plot3([p(1,i-1) p(1,i)], [p(2,i-1) p(2,i)], [p(3,i-1) p(3,i)], ...
            '-', 'Color', '#3B3A3A', 'LineWidth', link_girth(i-1))
        hold on

        % Plot the end-effector
        plot3(p(1,i), p(2,i), p(3,i), '*k', 'LineWidth', 4, ...
            'MarkerSize', joint_size, 'MarkerFaceColor', 'red', ...
            'MarkerEdgeColor', 'red')
        hold on
    end

    rectangle('Position', [-0.3 -0.3 0.6 0.6], 'FaceColor', 'black')
    hold on
end

hold off
grid on
grid minor
axis equal
view(view_angles(1),view_angles(2))
xlim([-1 1])
ylim([-1 1])
zlim([0 2])
xlabel('x')
ylabel('y')
zlabel('z')

end

