%% start file demonstrating the use of draw_robot
clear all;
clc;
load('start.mat');
set_figure('white',[0,90],0.4*[-1,1,-0.6,1,-0.5,0.5],'x[m]','y[m]','z[m]','3PRR robot')
Draw_Robot(B,A_in_w,E);
% B= 3x3 matrix containing vertices of base platform in columns
% A = 3x3 matrix containing vertices of moving platform in columns
% E = 3x3 matrix constaining coordinates of elbowsin columns
