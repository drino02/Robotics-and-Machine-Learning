% This script runs the algorithm for Resolved Rates Algorithm (RRA)

%% Clear Workspace
clc
clear
close all

%% Non-Algorithm-Specific Tuning Parameters
gamma = 1e-3;             % convergence criterion
max_iter = 10000;         % maximum iteration count for which beyond, 
                          % the solution is considered non-convergent

%% Tuning Parameters for RRA
delta_t = 99;
vmin = 0.03;
vmax = 0.1;
lambda_p = 5;

%% Constants and Initial Configuration
a = [1; 0.5; 0.4; 0.3; 0.2];  % column vector of link lengths
                              % [a1; a2; a3; a4; a5]
q_home = [0; 0; -90; 0; 0];   % initial joint configuration
                              % [q1; q2; q3; q4; q5] in degrees

%% Goal Position
%[goal1; goal2; ...; goaln]
p_goal = [0.75 0.2 0.3; ...
    0.4 0.4 0.4; ...
    0.6 0.5 0.5];

%% Resolved Rates Algorithm

% for animation output file
filename = strcat('RRA-sim-', datestr(datetime("now")));  
view = [45, 30];  % view angles for animation [azimuth, elevation]
[q_values, errors, iter_count, response_times, ret] = RRA(p_goal, a, ...
    delta_t, q_home, vmax, vmin, gamma, lambda_p, max_iter, view, filename);



