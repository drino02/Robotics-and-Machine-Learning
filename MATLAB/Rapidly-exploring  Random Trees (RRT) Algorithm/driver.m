clc;
clear;
close all;

N = 1000; % number of trials
start_state = [ 5 50];
goal_region = [90  0 100   0  100 100  90 100];
obstacles = [  5 10 15 10 15 20  5 20; % obstacle 1
              10 40 20 40 20 50 10 50; % obstacle 2
              20 70 30 70 30 80 20 80; % ...etc...
              30 20 40 20 40 30 30 30; 
              40 50 50 50 50 60 40 60;
              50  5 60  5 60 15 50 15;
              55 80 65 80 65 90 55 90;
              60 40 70 40 70 50 60 50;
              70 20 80 20 80 30 70 30
              75 65 85 65 85 75 75 75 ];
num_obstacles = size(obstacles,1);

%% OUTPUT

% Shortest path
path_shortest = [];
path_length_shortest = [];
P_final_shortestpath = [];
stdev_xx_yy_sp =[];

% Minimum Uncertainty
path_min_uncrtnty = [];
path_length_min_uncrtnty = [];
P_final_min_uncrtnty = [];
stdev_xx_yy_minu =[];

% Minimum Uncertainty
path_max_uncrtnty = [];
path_length_max_uncrtnty = [];
P_final_max_uncrtnty = [];
stdev_xx_yy_maxu =[];

%% PERFORM TRIALS
for i = 1:N
% CALL RRT FUNCTION
    [path,path_length] = RRT(start_state,goal_region,obstacles);
    
    % PROPAGATE KF
    [P_final, stdev_xx_yy] = propagate_KF_path(path,obstacles);

    if isempty(path_shortest) || (path_length_shortest > path_length)
        path_shortest = path;
        path_length_shortest = path_length;
        P_final_shortestpath = P_final;
        stdev_xx_yy_sp = stdev_xx_yy;
    end

    if isempty(path_min_uncrtnty) || ...
            (trace(P_final_min_uncrtnty) > trace(P_final))
        path_min_uncrtnty = path;
        path_length_min_uncrtnty = path_length;
        P_final_min_uncrtnty = P_final;
        stdev_xx_yy_minu = stdev_xx_yy;
    end

    if isempty(path_max_uncrtnty) || ...
            (trace(P_final_max_uncrtnty) < trace(P_final))
        path_max_uncrtnty = path;
        path_length_max_uncrtnty = path_length;
        P_final_max_uncrtnty = P_final;
        stdev_xx_yy_maxu = stdev_xx_yy;
    end
end

%% OUTPUT TO CONSOLE

% ==================SHORTEST PATH==================
fprintf('SHORTEST PATH:\n');
fprintf('\tPath Length = %f \n',path_length_shortest);
fprintf('\tTerminal State Uncertainty = %f\n', trace(P_final_shortestpath));

% ==================MIN UNCERTAINTY==================
fprintf('\nMINIMUM TERMINAL STATE UNCERTAINTY:\n');
fprintf('\tPath Length = %f \n',path_length_min_uncrtnty);
fprintf('\tTerminal State Uncertainty = %f\n', trace(P_final_min_uncrtnty));

% ==================MAX UNCERTAINTY==================
fprintf('\nMAXIMUM TERMINAL STATE UNCERTAINTY:\n');
fprintf('\tPath Length = %f \n',path_length_max_uncrtnty);
fprintf('\tTerminal State Uncertainty = %f\n', trace(P_final_max_uncrtnty));


%% GENERATE MAP WITH OBSTACLES (from generate_obstacles.m)

% ==================SHORTEST PATH==================
figure(1);
title('SHORTEST PATH');
hold on;
axis([0 100 0 100]);
box on;
path = path_shortest;
stdev_xx_yy = stdev_xx_yy_sp;

% plot the start state for the planning problem
plot(start_state(1),start_state(2),'.r','MarkerSize',20);

% plot the goal region for the planning problem
goal_x = [goal_region(1) goal_region(3) goal_region(5) goal_region(7)];
goal_y = [goal_region(2) goal_region(4) goal_region(6) goal_region(8)];
patch(goal_x,goal_y,'green');

% plot the locations of the obstacles
for i_obs = 1:num_obstacles
    obs_x = [obstacles(i_obs,1) obstacles(i_obs,3) ...
        obstacles(i_obs,5) obstacles(i_obs,7)];
    obs_y = [obstacles(i_obs,2) obstacles(i_obs,4) ...
        obstacles(i_obs,6) obstacles(i_obs,8)];
    patch(obs_x,obs_y,'blue');
end

% PATH PLOT
plot(path(end,1),path(end,2),'.k','Markersize', 20);
for i = 2:length(path)
    plot([path(i,1) path(i-1,1)],[path(i,2) path(i-1,2)],'-k','LineWidth',3);
end
hold on
for i = 1:length(path)
    pos = path(i,:);
    ellipse(stdev_xx_yy(i,1),stdev_xx_yy(i,2),0,pos(1),pos(2),'r');
    hold on
end
hold off

% ==================MIN UNCERTAINTY==================
figure(2); 
title('PATH OF MINIMUM TERMINAL STATE UNCERTAINTY');
hold on;
axis([0 100 0 100]);
box on;
path = path_min_uncrtnty;
stdev_xx_yy = stdev_xx_yy_minu;

% plot the start state for the planning problem
plot(start_state(1),start_state(2),'.r','MarkerSize',20);

% plot the goal region for the planning problem
goal_x = [goal_region(1) goal_region(3) goal_region(5) goal_region(7)];
goal_y = [goal_region(2) goal_region(4) goal_region(6) goal_region(8)];
patch(goal_x,goal_y,'green');

% plot the locations of the obstacles
for i_obs = 1:num_obstacles
    obs_x = [obstacles(i_obs,1) obstacles(i_obs,3) ...
        obstacles(i_obs,5) obstacles(i_obs,7)];
    obs_y = [obstacles(i_obs,2) obstacles(i_obs,4) ...
        obstacles(i_obs,6) obstacles(i_obs,8)];
    patch(obs_x,obs_y,'blue');
end

% PATH PLOT
plot(path(end,1),path(end,2),'.k','Markersize', 20);
for i = 2:length(path)
    plot([path(i,1) path(i-1,1)],[path(i,2) path(i-1,2)],'-k','LineWidth',3);
end
hold on
for i = 1:length(path)
    pos = path(i,:);
    ellipse(stdev_xx_yy(i,1),stdev_xx_yy(i,2),0,pos(1),pos(2),'r');
    hold on
end
hold off

% ==================MAX UNCERTAINTY==================
figure(3); 
title('PATH OF MAXIMUM TERMINAL STATE UNCERTAINTY');
hold on;
axis([0 100 0 100]);
box on;
path = path_max_uncrtnty;
stdev_xx_yy = stdev_xx_yy_maxu;

% plot the start state for the planning problem
plot(start_state(1),start_state(2),'.r','MarkerSize',20);

% plot the goal region for the planning problem
goal_x = [goal_region(1) goal_region(3) goal_region(5) goal_region(7)];
goal_y = [goal_region(2) goal_region(4) goal_region(6) goal_region(8)];
patch(goal_x,goal_y,'green');

% plot the locations of the obstacles
for i_obs = 1:num_obstacles
    obs_x = [obstacles(i_obs,1) obstacles(i_obs,3) ...
        obstacles(i_obs,5) obstacles(i_obs,7)];
    obs_y = [obstacles(i_obs,2) obstacles(i_obs,4) ...
        obstacles(i_obs,6) obstacles(i_obs,8)];
    patch(obs_x,obs_y,'blue');
end

% PATH PLOT
plot(path(end,1),path(end,2),'.k','Markersize', 20);
for i = 2:length(path)
    plot([path(i,1) path(i-1,1)],[path(i,2) path(i-1,2)],'-k','LineWidth',3);
end
hold on
for i = 1:length(path)
    pos = path(i,:);
    ellipse(stdev_xx_yy(i,1),stdev_xx_yy(i,2),0,pos(1),pos(2),'r');
    hold on
end
hold off




