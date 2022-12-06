function drawrobot(q)
%DRAWROBOT Draw the 4DOF Planar Arm 
%   Input:
%       q - configuration (thetas in degrees)
%   Output:
%       plot of the robotic arm pose

% run dirkin to generate frames
[frames, x_gripper] = dirkin(q);
hold on
% draw robot
DrawRobot_HW2p1(frames);
hold on;

% plot homeposition gripper center as a yellow star
plot(x_gripper(1),x_gripper(2),'pk', 'MarkerSize', 12, ...
    'MarkerFaceColor', 'y');
hold on
grid on
grid minor
xlabel('x')
ylabel('y')
xlim([-2 2.5])
ylim([-1 3.5])

end

