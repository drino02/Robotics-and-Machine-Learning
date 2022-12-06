%% Draw Puma 560: this function draws the puma robot.
% frames = 4:4:3 matrix containing the homogenous transpofrms for each
% local link frame with respect to the ground
% example of use:
%        DH_matrix=set_DH_matrix([0;0;0;0;0;0])
%        [frames] = direct_kinematics_using_DH(DH_matrix)
%        draw_puma560(frames,'surface')
%
% first line defines the DH_matrix for a given q vector
% second line calculates the direct kinematics and returns a 3D 4x4x3 matrix with frames T01, T02,...T06
% third line draws the robot

%last updated 8/14/2012 by Nabil Simaan



%
function draw_puma560(frames,action)

%% extracting all transformations T01, T02,...T06
for i=1:1:6
    eval(['T0',num2str(i),'= frames(:,:,',num2str(i),');']);
end;

draw_coordinate_system(0.9,eye(3),[0;0;0],['r' 'g' 'b'],'W');
%draw_coordinate_system(0.4,T01(1:3,1:3),T01(1:3,4),['r' 'g' 'b'],'1');
%draw_coordinate_system(0.3,T02(1:3,1:3),T02(1:3,4),['r' 'g' 'b'],'2');
%draw_coordinate_system(0.6,T03(1:3,1:3),T03(1:3,4),['r' 'g' 'b'],'3');
%draw_coordinate_system(0.3,T04(1:3,1:3),T04(1:3,4),['r' 'g' 'b'],'4');
%draw_coordinate_system(0.6,T05(1:3,1:3),T05(1:3,4),['r' 'g' 'b'],'5');
draw_coordinate_system(0.4,T06(1:3,1:3),T06(1:3,4),['r' 'g' 'b'],'6');

if strcmpi(action,'stick')
    frame_origins=[[0;0;0],T01(1:3,4),T02(1:3,4),T03(1:3,4),T04(1:3,4),T05(1:3,4),T06(1:3,4)];
    X_start=frame_origins(1,1:5); Y_start=frame_origins(2,1:5); Z_start=frame_origins(3,1:5);
    X_end=frame_origins(1,2:6); Y_end=frame_origins(2,2:6); Z_end=frame_origins(3,2:6);
    h=line([X_start;X_end],[Y_start;Y_end],[Z_start;Z_end]);
    set(h(1),'color','r');
    set(h(2),'color','g');
    set(h(3),'color','b');
    set(h(4),'color','c');
    set(h(5),'color','m');
    %set(h(6),'color','b');
end;
    


if strcmp(lower(action),'surface')
    
    
    %% Draw coordinate systems
    
    
    %% Draw Base
    load puma560_base_meters;
    n = length(base(3,:));
    C = ones(1,n)*0.5;
 %   figure(1);
    trisurf(t,base(1,:),base(2,:),base(3,:),C,'FaceColor',[0.3 0.3 0.3],'EdgeColor',[0.3 0.3 0.3],'FaceLighting','gouraud','EdgeLighting','phong','AmbientStrength', 0.15);
    %trisurf(t,base(1,:),base(2,:),base(3,:),C,'FaceColor',[0.3 0.3 0.3],'EdgeColor',[0.3 0.3 0.3]);
    %% Draw Link 1
    load 'puma560_link1_meters';
    R = [cos(pi) 0 sin(pi);0 1 0;-sin(pi) 0 cos(pi)];
    link1 = T01*link1;
    n = length(link1(3,:));
    C = ones(1,n)*0.5;
  %  figure(1);
  %  hold on;
    trisurf(t,link1(1,:),link1(2,:),link1(3,:),C,'FaceColor',[1 0 0],'EdgeColor',[0,0,0],'FaceLighting','gouraud','EdgeLighting','phong','AmbientStrength', 0.15);
    %trisurf(t,link1(1,:),link1(2,:),link1(3,:),C,'FaceColor',[1 0 0],'EdgeColor',[0,0,0]);
    %% Draw Link 2
    load puma560_link2_meters;
    n = length(link2(3,:));
    link2 = T02*link2;
    C = ones(1,n)*0.5;
   % figure(1);
    trisurf(t,link2(1,:),link2(2,:),link2(3,:),C,'FaceColor',[0 1 0],'EdgeColor',[0 1 0],'FaceLighting','gouraud','EdgeLighting','phong','AmbientStrength', 0.15);
    %trisurf(t,link2(1,:),link2(2,:),link2(3,:),C,'FaceColor',[0 1 0],'EdgeColor',[0 1 0]);
    %% Draw Link 3
    load puma560_link3_meters;
    link3 = T03*link3;
    n = length(link3(3,:));
    C = ones(1,n)*0.5;
   % figure(1);
    trisurf(t,link3(1,:),link3(2,:),link3(3,:),C,'FaceColor',[255/255,153/255,102/255],'EdgeColor',[255/255,153/255,102/255],'FaceLighting','gouraud','EdgeLighting','phong','AmbientStrength', 0.15);
    %trisurf(t,link3(1,:),link3(2,:),link3(3,:),C,'FaceColor',[255/255,153/255,102/255],'EdgeColor',[255/255,153/255,102/255]);
    %% Draw Link 4
    load puma560_link4_meters;
    link4 = T04*link4;
    n = length(link4(4,:));
    C = ones(1,n)*0.5;
   % figure(1);
    trisurf(t,link4(1,:),link4(2,:),link4(3,:),C,'FaceColor',[51/255,204/255,51/255],'EdgeColor',[51/255,204/255,51/255],'FaceLighting','gouraud','EdgeLighting','phong','AmbientStrength', 0.15);
    %trisurf(t,link4(1,:),link4(2,:),link4(3,:),C,'FaceColor',[51/255,204/255,51/255],'EdgeColor',[51/255,204/255,51/255]);
    
    %% Draw Link 5
    load puma560_link5_meters;
    link5 = T05*link5;
    n = length(link5(4,:));
    C = ones(1,n)*0.5;
   % figure(1);
    trisurf(t,link5(1,:),link5(2,:),link5(3,:),C,'FaceColor',[1,0,0],'EdgeColor',[1,0,0],'FaceLighting','gouraud','EdgeLighting','phong','AmbientStrength', 0.15);
    %trisurf(t,link5(1,:),link5(2,:),link5(3,:),C,'FaceColor',[1,0,0],'EdgeColor',[1,0,0]);
    %% Draw Link 6
    load puma560_link6_meters;
    link6 = T06*link6;
    n = length(link6(4,:));
    C = ones(1,n)*0.5;
   % figure(1);
    trisurf(t,link6(1,:),link6(2,:),link6(3,:),C,'FaceColor',[0 0 1],'EdgeColor',[0 0 1],'FaceLighting','gouraud','EdgeLighting','phong','AmbientStrength', 0.15);
    %trisurf(t,link6(1,:),link6(2,:),link6(3,:),C,'FaceColor',[0 0 1],'EdgeColor',[0 0 1]);
    %% Add a camera light, and tone down the specular highlighting
    camlight(-30,60);
    material('dull');
    
    %% Draw pen
    load markerpen_meters;
    pen = T06*pen;
    n = length(pen(4,:));
    C = ones(1,n)*0.5;
   % figure(1);
    trisurf(t,pen(1,:),pen(2,:),pen(3,:),C,'FaceColor',[226/255, 26/255, 91/255],'EdgeColor',[226/255, 26/255, 91/255],'FaceLighting','gouraud','EdgeLighting','phong','AmbientStrength', 0.15);
    %trisurf(t,link6(1,:),link6(2,:),link6(3,:),C,'FaceColor',[0 0 1],'EdgeColor',[0 0 1]);
    %% Add a camera light, and tone down the specular highlighting
    camlight(-30,60);
    material('dull');
    
end;


%%setting the view
%axis equal;
%view(144,16);