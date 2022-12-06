clear all;
close all;
clc;

figure(1);%
axis([-3,3,-.5,3])
axis equal;
axis manual;
grid on;
hold on; %needed to lock the limits of the current axis
% q_home = [45,60,70,60]/180*pi;
% [~,Frames] = RRRR_dirkin(q_home);
%load('RRRR_starter_files')
[aa,Frames] = dirkin([45 -60 -70 -60]);
DrawRobot_RRRR(Frames);