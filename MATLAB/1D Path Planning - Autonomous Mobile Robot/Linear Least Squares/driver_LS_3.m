clc;
clear;
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Some parameters you can use to set up your solution of SimLab 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

length_hallway = 10;
sensor_range = 0.5;

landmarks = [2; 5; 8];

num_landmarks = length(landmarks);

stdev_odometry = 0.1; % m/s
stdev_range = 0.01; % m

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robot's "Ground Truth" Trajectory when it travels the hallway
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

delta_t = 0.1; % time is discretized with a time-step of 0.1 seconds
v_robot = 0.1; % robot travels at a constant speed of 0.1 m/s

t_terminal = length_hallway/v_robot; % time that we reach the end of hall

t_vector = 0:delta_t:t_terminal; % time vector for robot's trajectory
x_vector = 0:v_robot*delta_t:length_hallway; % robot's true position in time
v_vector = ones(1,length(t_vector)).*v_robot;  % robot's true velocity in time

num_states = length(x_vector); % number of discrete-time states in trajectory

%% LEAST SQUARES SOLUTION; t = 0:100 seconds

l_count = 3;
N = 1000; % number of trials
n = length(t_vector); % number of steps
x_pos_overall_100 = []; % records of all positions across n-time steps
x_pos_overall_200 = []; % records of all positions across n-time steps
% and across N trials

%% populate the constant matrix A for t=0:100secs
A = [];
A_1 = [];
for j = 1:n
    if isempty(A_1)
        A_1 = [1 zeros(1,n+l_count-1)];
    else
        A_1(j,:) = [zeros(1,j-2) -1 1 zeros(1,n+l_count-j)];
    end
end
A = A_1;

A_2 = [];
index = 1;
for j = 1:n
    % detection of first landmark
    if t_vector(j) >= 15 && t_vector(j) <= 25
        A_2(index,:) = [zeros(1,j-1) -1 zeros(1,n-j) 1 0 0];
        index = index + 1;
    end
    
    % detection of 2nd landmark
    if t_vector(j) >= 45 && t_vector(j) <= 55
        A_2(index,:) = [zeros(1,j-1) -1 zeros(1,n-j) 0 1 0];
        index = index + 1;
    end
    
    % detection of 3rd landmark
    if t_vector(j) >= 75 && t_vector(j) <= 85
        A_2(index,:) = [zeros(1,j-1) -1 zeros(1,n-j) 0 0 1];
        index = index + 1;
    end
    
end

A = [A ; A_2];
pinv_A = pinv(A); % get the pseudoinverse of A for the first half

%% populate A for t = 100.1:200secs

A = [A; -A_1(2:end,:)];
A = [A; -A_2];
pinv_A_2 = pinv(A); % get the pseudoinverse of A for the second half


%% populate b vector for t=0:100secs
% LS Loop
for i = 1:N
    x_odom = (v_robot+stdev_odometry*randn(1,n))*delta_t;
    b = [0 x_odom(2:end)];
    
    k = length(b)+1;
    for j = 1:n
        % detection of first landmark
        if t_vector(j) >= 15 && t_vector(j) <= 25
            b(k) = (v_robot*(20-t_vector(j))+stdev_range*randn());
            k = k+1;
        end
        
        % detection of 2nd landmark
        if t_vector(j) >= 45 && t_vector(j) <= 55
            b(k) = (v_robot*(50-t_vector(j))+stdev_range*randn());
            k = k+1;
        end
        
        % detection of 3rd landmark
        if t_vector(j) >= 75 && t_vector(j) <= 85
            b(k) = (v_robot*(80-t_vector(j))+stdev_range*randn());
            k = k+1;
        end
        
    end
    X = pinv_A*b';
    x_pos_overall_100(:,i) = X;
    
%% populate b vector for t=100.1:200secs
    x_odom = (-v_robot+stdev_odometry*randn(1,n-1))*delta_t;
    b = [b x_odom];
    k = length(b)+1;
    for j = 1:n
        % detection of first landmark
        if t_vector(j) >= 15 && t_vector(j) <= 25
            b(k) = (-v_robot*(20-t_vector(j))+stdev_range*randn());
            k = k+1;
        end
        
        % detection of 2nd landmark
        if t_vector(j) >= 45 && t_vector(j) <= 55
            b(k) = (-v_robot*(50-t_vector(j))+stdev_range*randn());
            k = k+1;
        end
        
        % detection of 3rd landmark
        if t_vector(j) >= 75 && t_vector(j) <= 85
            b(k) = (-v_robot*(80-t_vector(j))+stdev_range*randn());
            k = k+1;
        end
    end
    
    X = pinv_A_2*b';
    x_pos_overall_200(:,i) = X;
end

x_pos_overall_100 = x_pos_overall_100(1:end-3,:);
x_pos_overall_200 = x_pos_overall_200(1:end-3,:);

%% mean absolute error plot
abs_err_100 = [];
abs_err_200 = [];
mean_abs_err_100 = [];
mean_abs_err_200 = [];
[m,n] = size(x_pos_overall_100);
for i = 1:n
    abs_err_100(:,i) = abs(x_vector' - x_pos_overall_100(1:end,i));
    abs_err_200(:,i) = abs(x_vector' - x_pos_overall_200(1:end,i));
end

for i = 1:m
    mean_abs_err_100(i) = mean(abs_err_100(i,:));
    mean_abs_err_200(i) = mean(abs_err_200(i,:));
    
end

line([t_vector t_vector(end)+t_vector(2:end)],[mean_abs_err_100 mean_abs_err_200(end-1:-1:1)],'Color','r');
grid on
grid minor
xlabel('time (seconds)');
ylabel('mean absolute error');


















