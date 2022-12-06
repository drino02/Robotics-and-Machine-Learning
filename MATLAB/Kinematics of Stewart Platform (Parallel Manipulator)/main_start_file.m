clc;
clear all
close all

load('sample.mat') %load the robot geometry at a sample pose

figure(1);
view(64,20);

%% for BB
Drawrobot_sg(A_wcs,B);
axis equal;
hold on; 
grid on;
