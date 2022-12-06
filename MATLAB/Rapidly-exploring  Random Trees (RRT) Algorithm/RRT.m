function [path,path_length] = RRT(start_state,goal_region,obstacles)
%RRT Rapidly Exploring Random Tree Algorithm
%   Input:
%       start_state = root node coordinate
%       goal_region = target goal region coordinates
%       obstacles = obstacles coordinates
%   Output:
%       path = matrix containing the coordinates of each path point
%       path_length = total path length

%% DEFINE CONSTANTS AND VARIABLES
% known map parameters
map_x_max = 100;
map_y_max = 100;

% growth distance
eps = 2;

% initialize tree with the root
Tree = start_state;
parents = 0;

%% PERFORM RRT ALGORITHM
while true
    % generate a random point
    rand_point = [(rand(1)*map_x_max) (rand(1)*map_y_max)];
    
    % check if rand pt is already a member of the tree
    ismember_check = find(Tree == rand_point);
    if ~isempty(ismember_check) && length(ismember_check) > 1 ...
            && ismember_check(2) == 2*ismember_check(1)
        continue;
    end
    
    % check the nearest neighbor
    nearest_nghbr_ind = nearest_neighbor(rand_point,Tree);
    nearest_nghbr = Tree(nearest_nghbr_ind,:);
    dist_to_nn = norm(rand_point-nearest_nghbr);

    % if dist to nearest neighbor > eps, interpolate and generate new point
    % otherwise, put the rand pt to the tree
    if dist_to_nn > eps
        theta = atan2((rand_point(2)-nearest_nghbr(2)),...
            (rand_point(1)-nearest_nghbr(1)));
        new_point_x = (cos(theta)*eps+nearest_nghbr(1));
        new_point_y = (sin(theta)*eps+nearest_nghbr(2));
        rand_point = [new_point_x new_point_y];
    end

    % check segment collision from rand pt to nearest neighbor
    if collision_check_segment(nearest_nghbr(1),nearest_nghbr(2),...
            rand_point(1),rand_point(2),obstacles) == 1
        continue;
    end

    % add new point to tree
    Tree(end+1,:) = rand_point;

    % assign parent indx to the rand pt
    parents(end+1,:) = nearest_nghbr_ind;
    
    % check if goal region has been reached
    last_node = Tree(end,:);
    if collision_check_point(last_node(1),last_node(2),goal_region)==1
        break;
    end

end

%% TRACE, EXTRACT, AND PLOT PATH
% set current node and its parent's index
current_node = Tree(end,:);
cur_parent_ind = parents(end,:);

% initialize path parameters
path =current_node;
path_length = 0;

% trace goal to start point
while cur_parent_ind ~= 0
    cur_parent_node = Tree(cur_parent_ind,:);
    path = [path; cur_parent_node];
    path_length = path_length + norm(current_node-cur_parent_node);
    current_node = Tree(cur_parent_ind,:);
    cur_parent_ind = parents(cur_parent_ind);
end

path = flipud(path);

