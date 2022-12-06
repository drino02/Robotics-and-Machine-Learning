function [top,left,bot,right] = sense(pos,obs)
% SENSE 
% Mimics Range Sensor. Can only sense obstacles vertically and horizontally
% at each step.
%
%   Input:
%       pos - current robot position vector (x,y)
%       obs_edges - known coordinates matrix of all obstacle edges
%   Output:
%       top - index of obstacle above robot within range
%       left - index of obstacle on the left of robot within range
%       bot - index of obstacle below robot within range
%       right - index of obstacle on the right of robot within range

% define sensor range
range = 5;

% initialize output vectors
right =[];
left =[];
top = [];
bot = [];

for i = 1:size(obs,1)
    if (norm(obs(i,1)-pos(1)) <= range) && pos(2) <= obs(i,6)...
            && pos(2) >= obs(i,2)
        right = 1;
    end

    if (norm(obs(i,3)-pos(1)) <= range)  && pos(2) <= obs(i,6)...
            && pos(2) >= obs(i,2)
        left = 1;
    end

    if (norm(obs(i,2)-pos(2)) <= range) && pos(1) <= obs(i,3)...
            && pos(1) >= obs(i,1)
        top = 1;
    end

    if (norm(obs(i,6)-pos(2)) <= range) && pos(1) <= obs(i,3)...
            && pos(1) >= obs(i,1)
        bot = 1;
    end
end
