close all; clear; clc;

% Read the data from CSV file
% T = readtable("./training/data/2024_02_03_14_22_02/positions_trim.csv");
% T(1:1500,:)

T_w = readtable("./training/data/250g/2024_11_06_17_21_07/positions.csv");
T_w = T_w(:,4:10);

T = readtable("./training/data/2024_02_03_14_22_02/positions_trim.csv");
T = T(1:1500,4:10);

% T_w = batch_transform(T,T_w);


% Extract X, Y, and Z data points
X = (T.x_end_avg - mean(T.x_end_avg)).*1000; 
Y = (T.y_end_avg - mean(T.y_end_avg)).*1000;
Z = (T.z_end_avg - min(T.z_end_avg)).*1000;

map = brewermap(9,'Set1');

figure(1); clf; hold on; grid on;

s = scatter3(X,Y,Z,'filled', 'MarkerFaceColor', '#509C49','MarkerEdgeColor','k','LineWidth',1);
s.SizeData=100;

% Extract X, Y, and Z data points
X_w = (T_w.x_end_avg - mean(T_w.x_end_avg)).*1000; 
Y_w = (T_w.z_end_avg - mean(T_w.z_end_avg)).*1000;
Z_w = (T_w.y_end_avg - min(T_w.y_end_avg)).*1000;

map = brewermap(9,'Set1');

figure(2); clf; hold on; grid on;

s = scatter3(X_w,Y_w,Z_w,'filled', 'MarkerFaceColor', 'r','MarkerEdgeColor','k','LineWidth',1);
s.SizeData=100;
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
% exportgraphics(gcf,'../figures/workspace/x-y.emf','ContentType', 'vector');



% Second tile for XZ plot
figure(3); hold on; axis equal; grid on;

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
% exportgraphics(gcf,'../figures/workspace/x-z.emf','ContentType', 'vector');
