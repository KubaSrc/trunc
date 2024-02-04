close all; clear; clc;

% Read the data from CSV file
T = readtable("./training/data/2024_01_29_13_12_45/positions.csv");

% Extract X, Y, and Z data points
X = (T.x_end_avg - mean(T.x_end_avg)).*1000; 
Y = (T.y_end_avg - mean(T.y_end_avg)).*1000;
Z = (T.z_end_avg - min(T.z_end_avg)).*1000;

map = brewermap(9,'Set1');

%% Visualize rotations from origin


% quaternion distance from mean
quat = [T.qx_end_avg,T.qy_end_avg,T.qz_end_avg,T.qw_end_avg];
end_point_quat = quaternion(quat);
end_point_quat_max = rad2deg(dist(end_point_quat,end_point_quat(1,:)));

% Home postion
x_h = T.x_end_avg(1).*1000; y_h = T.y_end_avg(1).*1000; z_h = T.z_end_avg(1).*1000;

% slice for manual inspection
sl = end_point_quat_max<3;

% create plot
fig = figure(1); clf; hold on; grid on;

x_w = 1000.*T.x_end_avg(sl);
y_w = 1000.*T.z_end_avg(sl);
z_w = 1000.*T.y_end_avg(sl);
x_w = x_w-x_w(1,:);
y_w = y_w-y_w(1,:); 
z_w = z_w-z_w(1,:); 

% scatter of points
s = scatter3(x_w,y_w,z_w,'filled','MarkerFaceColor',map(3,:),'MarkerFaceAlpha',1,'MarkerEdgeColor','k','MarkerEdgeAlpha',1,'LineWidth',1.25);
s.SizeData = 150;
xlabel('x');
ylabel('y');
zlabel('z');

% change view
el = 0;
view(0,el)


%% First tile for XY plot
figure(2); hold on; axis equal; grid on;
k = boundary(X, Y);
fill(X(k), Y(k), 'k', 'FaceColor', '#696969', 'FaceAlpha', 0.5, 'EdgeColor', '#696969','LineWidth',1.5);
s = scatter(X, Y, 'filled', 'MarkerFaceColor', '#509C49','MarkerEdgeColor','k','LineWidth',1);
s.SizeData=100;
xlim([-275,275]);
ylim([-275,275]);
hold off;

% Set the size of the axes lines and text
ax = gca; % Current axes
ax.FontSize = 14; % Change to desired font size
ax.LineWidth = 1.5; % Change to desired axis line width
set(gcf, 'Color', 'white');
set(gcf,'Units', 'inches','Position',[1,1,3,3])
exportgraphics(gcf,'../figures/workspace/x-y.emf','ContentType', 'vector');



% Second tile for XZ plot
figure(2); hold on; axis equal; grid on;
k = boundary(X, Z);
fill(X(k), Z(k), 'k', 'FaceColor', '#696969', 'FaceAlpha', 0.5, 'EdgeColor', '#696969','LineWidth',1.5);
s=scatter(X, Z, 'filled', 'MarkerFaceColor', '#509C49','MarkerEdgeColor','k','LineWidth',1);
s.SizeData=100;
hold off;
xlim([-275,275]);
ylim([-25,225]);

% Set the size of the axes lines and text
ax = gca; % Current axes
ax.FontSize = 14; % Change to desired font size
ax.LineWidth = 1.5; % Change to desired axis line width
set(gcf, 'Color', 'white');
set(gcf,'Units', 'inches','Position',[1,1,3,3])
exportgraphics(gcf,'../figures/workspace/x-z.emf','ContentType', 'vector');