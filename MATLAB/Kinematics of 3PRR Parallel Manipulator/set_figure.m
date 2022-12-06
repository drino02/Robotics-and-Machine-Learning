function [] = set_figure(fig_bk_color,view_direction,axis_vec,x_label,y_label,z_label,fig_title)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Sample call: set_figure('white',[15,12],[-0.2,0.2,-0.2,0.2,-0.5,0.5],'x[m]','y[m]','Phi[deg]','Robot workspace')

hfig=gcf;
set(hfig,'color',fig_bk_color);         % bakcground color %
if get(hfig,'color')==[1 1 1],          % white background %
    set(gcf,'DefaultTextColor','Black')  % black text       %
else                            % black background %
    get(hfig,'color')==[0,0,0];  % white text       %
end;
hold on ;
view(view_direction(1),view_direction(2));
hfig=gcf;
axis(axis_vec); 
haxis=gca;

if get(hfig,'color')==[1 1 1],
    set(haxis,'xcolor','black');
    set(haxis,'ycolor','black');
    set(haxis,'zcolor','black');
else
    set(haxis,'xcolor','white');
    set(haxis,'ycolor','white');
    set(haxis,'zcolor','white');
end;

xlabel(x_label);
ylabel(y_label);
zlabel(z_label);
set(haxis,'xgrid','on');
set(haxis,'ygrid','on');
set(haxis,'zgrid','on');
title(fig_title);
end

