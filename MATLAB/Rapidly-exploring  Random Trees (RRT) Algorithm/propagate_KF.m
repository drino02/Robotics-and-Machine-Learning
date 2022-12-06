function P_k_next = propagate_KF(A,H_k,Q,R_k,P_k)
%PROPAGATE_KF Computes the Next Value of the Error Covariance Matrix
%   Input:
%       A - constant state correlation matrix
%       H_k - measurement matrix
%       Q - process noise covariance matrix
%       R_k - white sensor noise covariance matrix
%       P_k - error covariance matrix

P_k_next = ((A*P_k*A'+Q)\eye(size(Q)) + H_k'*(R_k\H_k))\eye(size(P_k));

end



