function [P_final, stdev_xx_yy] = propagate_KF_path(path,obstacles)
%PROPAGATE_KF_PATH Propagates Kalan Filter Through Entire Path
%   Input:
%       path - path matrix
%       obstacles - obstacle edges matrix
%   Output:
%       P_final - final terminal error covariance matrix
%       stdev_xx_yy - standard deviations of xx and yy elements of P_k

dt = 1;
P_final = [];
stdev_xx_yy = [];

A = [1 0 0 0;
     0 1 0 0;
     dt 0 1 0;
     0 dt 0 1];

% for i = 1:count_edges
%     A(end+1,:) = [0 0 0 0 zeros(1,i-1) 1 zeros(1,count_edges-i) zeros(1,count_edges)];
% end
% 
% for i = 1:count_edges
%     A(end+1,:) = [0 0 0 0 zeros(1,count_edges) zeros(1,i-1) 1 zeros(1,count_edges-i)];
% end

P_k = eye(size(A));
Q = zeros(size(A));
Q(1,1) = 1;
Q(2,2) = 1;
Q(3,3) = 1;
Q(4,4) = 1;
stdev_xx_yy = [sqrt(P_k(3,3)) sqrt(P_k(4,4))];


for i = 2:length(path)

    [top,left,bot,right] = sense(path(i,:),obstacles);

    H_k = [1 0 0 0;
           0 1 0 0];
    R_k = [1 0; 0 1];
    
    if ~isempty(top)
        H_k(end+1,:) = [0 0 0 1];
        temp_sz_Rk = size(R_k,1);
        R_k(:,end+1) = zeros(temp_sz_Rk,1);
        R_k(end+1,:) = [zeros(1,temp_sz_Rk) 1];
    end

    if ~isempty(bot)
        H_k(end+1,:) = [0 0 0 1];
        temp_sz_Rk = size(R_k,1);
        R_k(:,end+1) = zeros(temp_sz_Rk,1);
        R_k(end+1,:) = [zeros(1,temp_sz_Rk) 1];
    end

    if ~isempty(left)
        H_k(end+1,:) = [0 0 1 0];
        temp_sz_Rk = size(R_k,1);
        R_k(:,end+1) = zeros(temp_sz_Rk,1);
        R_k(end+1,:) = [zeros(1,temp_sz_Rk) 1];
    end

    if ~isempty(right)
        H_k(end+1,:) = [0 0 1 0];
        temp_sz_Rk = size(R_k,1);
        R_k(:,end+1) = zeros(temp_sz_Rk,1);
        R_k(end+1,:) = [zeros(1,temp_sz_Rk) 1];
    end
    
    P_k = propagate_KF(A,H_k,Q,R_k,P_k);
    
    stdev_xx_yy = [stdev_xx_yy; sqrt(P_k(3,3)) sqrt(P_k(4,4))];
end

P_final = P_k;