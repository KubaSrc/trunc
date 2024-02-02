%% Global setup

restoredefaultpath;

close all; clear all; clc
warning('off','all')

% fig sizes and scale factor
fig_w = 300; fig_h = 300; fig_s = 5;

% fonts
ax_font_size = 7*fig_s;
set(0,'DefaultTextFontname', 'CMU Sans Serif' )
set(0,'DefaultAxesFontName', 'CMU Sans Serif' )

% colors
map = brewermap(9,'Set1');

% save path
export_fig = false;

%% Load in data

T = readtable("./repeatability/2024_01_24_21_00_43/positions.csv");
% Flip z and y
z = T.z_end_avg; y = T.y_end_avg;
T.y_end_avg = z; T.z_end_avg = y;
n_points = 100;

% find distance from mean, averge distance error, and distance variance
end_point = 1000.*[reshape(T.x_end_avg,n_points,1,[]),reshape(T.z_end_avg,n_points ,1,[]),reshape(T.y_end_avg,n_points ,1,[])];
end_point_mean = mean(end_point,3);
end_point_d = sqrt(sum((end_point-end_point_mean).^2,2));
e_point_d = abs((end_point-end_point_mean));
end_point_mean_e = mean(end_point_d,3);
end_point_var_e =  mean(end_point_d,3);

% quaternion distance from mean
quat = [reshape(T.qx_end_avg,n_points,1,[]),reshape(T.qy_end_avg,n_points ,1,[]),reshape(T.qz_end_avg,n_points ,1,[]),reshape(T.qw_end_avg,n_points ,1,[])];
end_point_quat = repmat(quaternion(1, 0, 0, 0), size(quat,1), size(quat,3));

% just matlab conversion from array to object
for ii=1:size(quat,1)
    for jj = 1:size(quat,3)
        end_point_quat(ii,jj) = quaternion(quat(ii,:,jj));
    end
end

% use built in methods to find rotational distances
end_point_quat_mean = meanrot(end_point_quat,2);
end_point_quat_d = rad2deg(dist(end_point_quat,end_point_quat_mean));
end_point_quat_mean_e = mean(end_point_quat_d,2);


%% Figure 2.1

fig = figure(21); clf; hold on; grid on

% Plot data
s = scatter3(end_point_mean(:,1)-mean(end_point_mean(:,1)),end_point_mean(:,2)-mean(end_point_mean(:,2)),end_point_mean(:,3)-mean(end_point_mean(:,3)),[],end_point_mean_e,'filled','MarkerEdgeColor',"k",'LineWidth',1.25);
s.SizeData = 150;
plot3(end_point_mean(:,1)-mean(end_point_mean(:,1)),end_point_mean(:,2)-mean(end_point_mean(:,2)),end_point_mean(:,3)-mean(end_point_mean(:,3)),'Color',[0, 0, 0, 0.3],'LineWidth',2)

% Set colormap and add color bar
colormap(gca,flipud(brewermap([],'Spectral')));
cb = colorbar('northoutside','LineWidth',1.5,'FontSize',ax_font_size);

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1.25;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);
view([45,37.5])

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);

% export fig
if export_fig
    exportgraphics(gcf,'../Figures/Repetability/pos_error.png','Resolution',300*fig_s)
end



%% Figure 2.2

fig = figure(22); clf; hold on; grid on

% Plot data
s = scatter3(end_point_mean(:,1)-mean(end_point_mean(:,1)),end_point_mean(:,2)-mean(end_point_mean(:,2)),end_point_mean(:,3)-mean(end_point_mean(:,3)),[],end_point_quat_mean_e,'filled','MarkerEdgeColor',"k",'LineWidth',1.25);
s.SizeData = 150;
plot3(end_point_mean(:,1)-mean(end_point_mean(:,1)),end_point_mean(:,2)-mean(end_point_mean(:,2)),end_point_mean(:,3)-mean(end_point_mean(:,3)),'Color',[0, 0, 0, 0.3],'LineWidth',2)

% Set colormap and add color bar
colormap(gca,flipud(brewermap([],'RdBu')));
cb = colorbar('northoutside','LineWidth',1.5,'FontSize',ax_font_size);

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1.25;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);
view([45,37.5])

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);

% export fig
if export_fig
    exportgraphics(gcf,'../Figures/Repetability/quat_error.png','Resolution',300*fig_s)
end


%% Figure 2.3

fig = figure(23); clf; hold on;

end_point_d_flat = reshape(end_point_d,1,[]);
histogram(end_point_d_flat,'facecolor',map(1,:),'facealpha',.2,'EdgeColor','none')
histogram(end_point_d_flat,'EdgeColor',map(1,:),'linewidth',3,'DisplayStyle','stairs')

% PRINT OUT ENDPOINT
mean(end_point_d_flat)

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1.25;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
xlim("auto")

% export fig
if export_fig
    exportgraphics(gcf,'../Figures/Repetability/pos_hist.png','Resolution',300*fig_s)
end


%% Figure 2.4

fig = figure(24); clf; hold on;

histogram(end_point_quat_d,'facecolor',map(2,:),'facealpha',.2,'EdgeColor','none')
histogram(end_point_quat_d,'EdgeColor',map(2,:),'linewidth',3,'DisplayStyle','stairs')

% PRINT OUT ENDPOINT
mean(reshape(end_point_quat_d,1,[]))

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1.25;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
xlim("auto")

% export fig
if export_fig
    exportgraphics(gcf,'../Figures/Repetability/quat_hist.png','Resolution',300*fig_s)
end