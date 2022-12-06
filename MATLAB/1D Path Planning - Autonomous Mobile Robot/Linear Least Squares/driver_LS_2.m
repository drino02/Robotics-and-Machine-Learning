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

%% LEAST SQUARES SOLUTION

l_count = 3;
N = 1000; % number of trials
n = length(t_vector); % number of steps
x_pos_overall = []; % records of all positions across n-time steps 
                    % and across N trials

% populate the constant matrix A
A = [];

for j = 1:n
    if isempty(A)
        A = [1 zeros(1,n+l_count-1)];
    else
        A(j,:) = [zeros(1,j-2) -1 1 zeros(1,n+l_count-j)];
    end
end

index = 1;
for j = 1:n
    % detection of first landmark
    if t_vector(j) >= 15 && t_vector(j) <= 25
        A(n+index,:) = [zeros(1,j-1) -1 zeros(1,n-j) 1 0 0];
        index=index+1;
    end

    % detection of 2nd landmark
    if t_vector(j) >= 45 && t_vector(j) <= 55
        A(n+index,:) = [zeros(1,j-1) -1 zeros(1,n-j) 0 1 0];
        index=index+1;
    end

    % detection of 3rd landmark
    if t_vector(j) >= 75 && t_vector(j) <= 85
        A(n+index,:) = [zeros(1,j-1) -1 zeros(1,n-j) 0 0 1];
        index=index+1;
    end
   
end

% get the pseudoinverse of A
pinv_A = pinv(A);

% LS Loop
for i = 1:N
    x_odom = (v_robot+stdev_odometry*randn(1,1001))*delta_t;
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
    x_pos_overall(:,i) = X;
end


%% mean absolute error plot
abs_err = [];
mean_abs_err = [];
[m,n] = size(x_pos_overall);
for i = 1:n
    abs_err(:,i) = abs(x_vector' - x_pos_overall(1:end-3,i));
end

for i = 1:m-3
    mean_abs_err(i) = mean(abs_err(i,:));
end

line(t_vector,mean_abs_err(:),'Color','r');
grid on
grid minor
xlabel('time (seconds)');
ylabel('mean absolute error');


















