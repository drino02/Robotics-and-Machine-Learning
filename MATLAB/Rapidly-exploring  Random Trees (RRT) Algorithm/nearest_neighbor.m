function nearest_nghbr_ind = nearest_neighbor(rand_point,Tree)
% NEAREST_NEIGHBOR
% Finds the index of the nearest neighbor to the input random point from
% the Tree.
%
%   Input:
%       rand_point - point whose nearest neighbor is being located
%   Output:
%       nearest_nghbr_ind - index of the nearest neighbor form the tree


% loop variable initializers
% set values to very large numbers
nearest_nghbr = [inf inf]; 
nearest_nghbr_ind = inf;

% check the nearest neighbor from the tree
row_count = size(Tree,1);
for i = 1:row_count
    dist_prev = norm(rand_point - nearest_nghbr);
    dist_cur = norm(rand_point - Tree(i,:));
    if dist_cur < dist_prev
        nearest_nghbr_ind = i;
    end
    nearest_nghbr = Tree(nearest_nghbr_ind,:);
end


