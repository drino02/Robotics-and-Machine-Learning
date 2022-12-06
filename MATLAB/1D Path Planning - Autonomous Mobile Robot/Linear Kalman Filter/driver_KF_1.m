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

N = 1000; % number of trials



%% Define A and initial states
A = [1 0;
    delta_t 1];

x0 = 0;
P0 = [1 0;
    0 0.0001];

Q = [10^(-8) 0;
    0 0];

% sensor noise covariance matrices
R = stdev_odometry^2;

x_pos_overall = []; % all x positions across all time steps per trial
stddev_xk_err = []; % std. dev. of position estimate's error across all
                    % time steps


for j = 1:N
    % x positions via odometry measurements with noise
    odom =[0];
    for i = 2:length(t_vector)
        odom(i) = v_robot+stdev_odometry*randn();
    end
    
    % set initial values
    x_prev = [v_robot x0]';
    P_prev= P0;
    
    % all positions per trial
    x_pos = [x_prev(2)];
    
    % for plotting stddev of position estimate's error
    if j== N
        stddev_xk_err(1) = sqrt(P_prev(2,2));
    end
    
    for i = 2:length(t_vector)
        % ===============Time Update================
        xk_hat_pred = A*x_prev;
        Pk_pred = A*P_prev*A' + Q;
        
        % ============Measurement Update=============
        H = [1 0];
        z = [odom(i)]';
        
        K = Pk_pred*H'*pinv(H*Pk_pred*H'+R);
        
        % Update prev values
        x_prev = xk_hat_pred + K*(z - H*xk_hat_pred);
        P_prev = (eye(2,2) - K*H)*Pk_pred;
        
        % update position values
        x_pos(i) = x_prev(2);
        
         % for plotting stddev of position estimate's error
        if j == N
            stddev_xk_err(i) = sqrt(P_prev(2,2));
        end
    end
    
    x_pos_overall(:,j) = x_pos';
    
end

%% mean absolute error plot
abs_err = [];
mean_abs_err = [];
[m,n] = size(x_pos_overall);
for i = 1:n
    abs_err(:,i) = abs(x_vector' - x_pos_overall(:,i));
end

for i = 1:m
    mean_abs_err(i) = mean(abs_err(i,:));
end

plt1 = line(t_vector,mean_abs_err(:),'Color','r');
hold on
plt2 = line(t_vector,stddev_xk_err,'Color','b');
hold off
grid on
grid minor
legend([plt1 plt2], 'Mean absolute error', 'Std. Deviation (\sigma_x_x_k)')
xlabel('time (seconds)');
ylabel('mean absolute error  & std. deviation');

