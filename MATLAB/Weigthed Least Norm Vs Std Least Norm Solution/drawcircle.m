function [x,y] = drawcircle(varargin)
%CYLINDER Generate cylinder.
%   [X,Y] = drawcircle(radius,origin,n,colorstring) forms a circle based on the radius
%   r centered about point "origin"  The circle has n points around the circumference.
%   fill(x,y,colorstring);
% You can call this function with 1 to 4 input argumants:
%  [X,Y]=drawcircle(r)
%  [X,Y]=drawcircle(r,origin)
%  [X,Y]=drawcircle(r,origin,n)
%  [X,Y]=drawcircle(r,origin,n,colorstring)
%  Example of use: [X,Y]=drawcircle(2,[1,2],20,'y') draws a yellow circle with
%  radius of 2 units centered at point [1,2].

error(nargchk(0,4,nargin));  %check that no more than 4 inputs have been provided
n_default = 20; %number of points on circumference
r_default = 1;  %default radius
origin_default=[0;0]; %setting up the default origin for the circle center
colorstring_default='red'; %set default color

if nargin==1, %[X,Y,Z]=drawcircle(r)
    r=varargin{1};
    origin=[0;0]; %setting up the default origin for the circle center
    n = 20; %number of points on circumference
    colorstring='red'; %set default color
end; %if called with circle(origin,...)
if nargin==2, %[X,Y,Z]=drawcircle(r,origin)
    r=varargin{1};
    origin = varargin{2};
    n = 20; %number of points on circumference
    colorstring='red'; %set default color
end;
if nargin==3, %[X,Y,Z]=drawcircle(r,origin,n)
    r=varargin{1};
    origin = varargin{2};
    n = varargin{3}; %number of points on circumference
    colorstring='red'; %set default color
end
if nargin==4, %[X,Y,Z]=drawcircle(r,origin,n,colorstring)
    r=varargin{1};
    origin = varargin{2};
    n = varargin{3}; %number of points on circumference
    colorstring=varargin{4}; %set default color
end;
theta = (0:n)/n*2*pi;  %vector of polar angles of points along circumfernce
x = r*cos(theta);
y = r*sin(theta);

x_offset=origin(1)*ones(1,length(theta));
y_offset=origin(2)*ones(1,length(theta));

x=x+x_offset;
y=y+y_offset;

fill(x,y,colorstring); %draw the circle

end
