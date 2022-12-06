% This script runs the algorithm for Pure Inverse Kinematics (PIK)

%% Clear Workspace
clc
clear
close all

%% Non-Algorithm-Specific Tuning Parameters
gamma = 1e-2;             % convergence criterion
max_iter = 10000;         % maximum iteration count for which beyond, 
                          % the solution is considered non-convergent

%% Tuning Parameters ofr PIK
step = 0.1;                 % PIK loop step size
guess = [0; 0; 0; 0; 0];  % column vector of the initial guess values 
                          % for the joint variables q1-q5

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

%% Pure Inverse Kinematics

% for animation output file
datetime.setDefaultFormats('default','yyyy-MM-dd hh:mm:ss')
filename = strcat('PIK-sim-', datestr(datetime("now")));  
view = [45, 30];  % view angles for animation [azimuth, elevation]

[q_values, errors, iter_count, response_times, ret] = InvKin(p_goal, a, gamma, ...
     step, q_home, max_iter, view, filename);



