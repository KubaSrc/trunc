close all; clc; clear all;

syms theta_1 theta_2 theta_3 theta_4 theta_5 theta_6 L d_tool

% Shoulder
T_shoulder_local = Segment_transform(theta_1,theta_2,-2.*L./7);
T_shoulder = T_shoulder_local;

% Elbow
T_elbow_local = Segment_transform(theta_3,theta_4,-3.*L./7);
T_elbow = T_shoulder*T_elbow_local;

% Wrist
T_wrist_local = Segment_transform(theta_5,theta_6,-2.*L./7);
T_wrist = T_elbow*T_wrist_local;

% Tool
T_tool_local = Segment_transform(0,0,-d_tool);
T_tool = T_wrist*T_tool_local;


%% Visualize arm config

% Shoulder
theta_1_n = deg2rad(0);
theta_2_n = deg2rad(-30);

% Elbow
theta_3_n = deg2rad(0);
theta_4_n = deg2rad(30);

% Wrist
theta_5_n = deg2rad(0);
theta_6_n = deg2rad(30);

% Lengths
L_n = 710;
d_tool_n = 100;

% Numerical substituion
T_shoulder_n = double(subs(T_shoulder,[theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, L, d_tool],...
                    [theta_1_n, theta_2_n, theta_3_n, theta_4_n, theta_5_n, theta_6_n, L_n, d_tool_n]));

T_elbow_n = double(subs(T_elbow,[theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, L, d_tool],...
                    [theta_1_n, theta_2_n, theta_3_n, theta_4_n, theta_5_n, theta_6_n, L_n, d_tool_n]));

T_wrist_n = double(subs(T_wrist,[theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, L, d_tool],...
                    [theta_1_n, theta_2_n, theta_3_n, theta_4_n, theta_5_n, theta_6_n, L_n, d_tool_n]));

T_tool_n = double(subs(T_tool,[theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, L, d_tool],...
                    [theta_1_n, theta_2_n, theta_3_n, theta_4_n, theta_5_n, theta_6_n, L_n, d_tool_n]));

% Visualize arm
[s_shoulder,s_elbow,s_wrist] = draw_arm(T_shoulder_n,T_elbow_n,T_wrist_n,T_tool_n);


%% Basic control of arm

addpath('../util/NatNet_SDK_4.1/NatNetSDK/Samples/Matlab');
addpath('../util/')

%% Helper functions

function T = Segment_transform(theta_1,theta_2,d)
Rx = [1,0,0,0;
      0,cos(theta_1),-sin(theta_1),0;
      0,sin(theta_1),cos(theta_1),0;
      0,0,0,1];
Ry = [cos(theta_2),0,sin(theta_2),0;
      0,1,0,0;
      -sin(theta_2),0,cos(theta_2),0;
      0,0,0,1];
Tz = [1,0,0,0;
      0,1,0,0;
      0,0,1,d;
      0,0,0,1];
T = Tz*Rx*Ry;
end

% Helper function for drawing arm
function [s_shoulder,s_elbow,s_wrist] = draw_arm(T_shoulder_n,T_elbow_n,T_wrist_n,T_tool_n)
    % Plot of the arm
    figure(1); clf; hold on
    set(gcf, 'Color', 'white');

    % Plot triangles
    tri_origin = [0,65.*sin(pi/3),-65.*sin(pi/3),0,;
                65,-65.*cos(pi/3),-65.*cos(pi/3),65;
                0,0,0,0;
                1,1,1,1];
    tri_shoulder = T_shoulder_n*tri_origin;
    tri_elbow = T_elbow_n*tri_origin;
    tri_wrist = T_wrist_n*tri_origin;

    plot3(tri_origin(1,:),tri_origin(2,:),tri_origin(3,:),'Color',[0.5,0.5,0.5,0.5],'LineWidth',3)
    plot3(tri_shoulder(1,:),tri_shoulder(2,:),tri_shoulder(3,:),'Color',[0.5,0.5,0.5,0.5],'LineWidth',3)
    plot3(tri_elbow(1,:),tri_elbow(2,:),tri_elbow(3,:),'Color',[0.5,0.5,0.5,0.5],'LineWidth',3)
    plot3(tri_wrist(1,:),tri_wrist(2,:),tri_wrist(3,:),'Color',[0.5,0.5,0.5,0.5],'LineWidth',3)

    % Plot of cables
    plot3([tri_origin(1,1),tri_shoulder(1,1),tri_elbow(1,1),tri_wrist(1,1)],...
          [tri_origin(2,1),tri_shoulder(2,1),tri_elbow(2,1),tri_wrist(2,1)],...
          [tri_origin(3,1),tri_shoulder(3,1),tri_elbow(3,1),tri_wrist(3,1)],'Color','k','LineWidth',2)
    plot3([tri_origin(1,2),tri_shoulder(1,2),tri_elbow(1,2),tri_wrist(1,2)],...
          [tri_origin(2,2),tri_shoulder(2,2),tri_elbow(2,2),tri_wrist(2,2)],...
          [tri_origin(3,2),tri_shoulder(3,2),tri_elbow(3,2),tri_wrist(3,2)],'Color','k','LineWidth',2)
    plot3([tri_origin(1,3),tri_shoulder(1,3),tri_elbow(1,3),tri_wrist(1,3)],...
          [tri_origin(2,3),tri_shoulder(2,3),tri_elbow(2,3),tri_wrist(2,3)],...
          [tri_origin(3,3),tri_shoulder(3,3),tri_elbow(3,3),tri_wrist(3,3)],'Color','k','LineWidth',2)

    % Find segment lengths
    s_shoulder = sqrt(sum((tri_shoulder(1:3,1:3)-tri_origin(1:3,1:3)).^2,1));
    s_elbow = sqrt(sum((tri_elbow(1:3,1:3)-tri_shoulder(1:3,1:3)).^2,1));
    s_wrist = sqrt(sum((tri_wrist(1:3,1:3)-tri_elbow(1:3,1:3)).^2,1));

    % Plot joints
    r = 30;
    [X, Y, Z] = sphere(100);
    X = r * X; Y = r * Y; Z = r * Z;
    
    surf(X+T_shoulder_n(1,4), Y+T_shoulder_n(2,4), Z+T_shoulder_n(3,4),'FaceColor','r','EdgeColor','none');
    surf(X+T_elbow_n(1,4), Y+T_elbow_n(2,4), Z+T_elbow_n(3,4),'FaceColor','r','EdgeColor','none');
    surf(X+T_wrist_n(1,4), Y+T_wrist_n(2,4), Z+T_wrist_n(3,4),'FaceColor','r','EdgeColor','none');
    
    % Plot of links
    plot3([0,T_shoulder_n(1,4)],[0,T_shoulder_n(2,4)],[0,T_shoulder_n(3,4)],'LineWidth',4,'Color','k')
    plot3([T_shoulder_n(1,4),T_elbow_n(1,4)],[T_shoulder_n(2,4),T_elbow_n(2,4)],[T_shoulder_n(3,4),T_elbow_n(3,4)],'LineWidth',4,'Color','k')
    plot3([T_elbow_n(1,4),T_wrist_n(1,4)],[T_elbow_n(2,4),T_wrist_n(2,4)],[T_elbow_n(3,4),T_wrist_n(3,4)],'LineWidth',4,'Color','k')
    plot3([T_wrist_n(1,4),T_tool_n(1,4)],[T_wrist_n(2,4),T_tool_n(2,4)],[T_wrist_n(3,4),T_tool_n(3,4)],'LineWidth',4,'Color','k')
    
    % Visualize the origin triad
    hold on; % Keep the current plot
    quiver3(0, 0, 0, 100, 0, 0, 'r',LineWidth=3); % X-axis in red
    quiver3(0, 0, 0, 0, 100, 0, 'b',LineWidth=3); % Y-axis in green
    quiver3(0, 0, 0, 0, 0, 100, 'g',LineWidth=3); % Z-axis in blue
    hold off;
    
    % Adjust the camera view
    view(30, 30); % Set the azimuth and elevation
    
    % Other plot settings
    axis equal; % Equal scaling for all axes
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    % Add lighting to enhance 3D effect
    camlight left;
    lighting phong;
    grid on;
    
    xlim([-500, 500])
    ylim([-500, 500])
    zlim([-800 100])
    
    set(gcf, 'Position', [100, 100, 800, 800]);
    ax = gca;
    ax.FontSize = 14;
    ax.LineWidth = 2;
end