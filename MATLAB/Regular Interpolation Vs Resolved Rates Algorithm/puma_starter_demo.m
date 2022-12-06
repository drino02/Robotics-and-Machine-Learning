clc;
close all;
clear all;
figure(1)
view(3)
axis([-1,1,-1,1,0,1.8]);
axis equal
grid on; 

[frames, p, R] = DirKin((pi/180)*[-45;60;45;45;30;45])

draw_puma560(frames,'surface')
%draw_puma560(frames,'stick')