function drawrobot(q)
%DRAWROBOT Draw the 4DOF Planar Arm 
%   Input:
%       q - configuration (thetas in degrees)
%   Output:
%       plot of the robotic arm pose

% run dirkin to generate frames
[~,frames] = dirkin(q);

% joint positions vector
pos = [0;0];
for i=1:length(frames)
    pos = [pos frames(1:2,4,i)]; 
end

% plot
for i = 1:length(pos)-1
    % plot frame 0 (world axis)
    plot([0 0.5],[0 0],'->','color','#808080','LineWidth', 2,'MarkerSize',5)
    plot([0 -0.5],[0 0],'-<','color','#808080','LineWidth', 2,'MarkerSize',5);
    plot([0 0],[0 0.5],'-^','color','#808080','LineWidth', 2,'MarkerSize',5);
    plot([0 0],[0 -0.5],'-v','color','#808080','LineWidth', 2,'MarkerSize',5);
    
    % plot the arm
    plot([pos(1,i) pos(1,i+1)],[pos(2,i) pos(2,i+1)], '-ok', 'LineWidth', ...
        3,'MarkerSize', 8, 'MarkerFaceColor', 'blue')
    if(i == length(pos)-1)
        plot(pos(1,i+1),pos(2,i+1), 'pk','MarkerSize', 20, ...
            'MarkerFaceColor', 'y')
    end
    
    text(0.5,-0.1,'x_0')
    text(-0.1,0.5,'y_0')
    hold on
end
hold off
grid on
xlabel('x')
ylabel('y')
xlim([-5 5])
ylim([-2 8])

end

