%% Global setup

close all; clear all; clc
warning('off','all')

% fig sizes and scale factor
fig_w = 300; fig_h = 300; fig_s = 3;

% fonts
ax_font_size = 9*fig_s;
legend_font_size = 9*fig_s;
set(0,'DefaultTextFontname', 'CMU Sans Serif' )
set(0,'DefaultAxesFontName', 'CMU Sans Serif' )

% colors
map = brewermap(9,'Set1');

% save path
export_fig = true;


%% Load-in all data

T_flex = readtable("./Instron Data/flex-shaft.csv");

T_spf = readtable("./Instron Data/simple-printed-force.csv");
T_spt_a = readtable("./Instron Data/simple-printed-bending.csv");
T_spt_p = readtable("./Instron Data/simple-printed-torsion.csv");

T_ssf = readtable("./Instron Data/simple-steel-force.csv");
T_sst_a = readtable("./Instron Data/simple-steel-bending.csv");
T_sst_p = readtable("./Instron Data/simple-steel-torsion.csv");

T_tsf = readtable("./Instron Data/complex-steel-force.csv");
T_tst_a = readtable("./Instron Data/complex-steel-bending.csv");
T_tst_p = readtable("./Instron Data/complex-steel-torsion.csv");


%% Flex shaft plot

fig = figure(1); clf; hold on;

plot(T_flex.Strain_0*10,T_flex.Stifness_0,'LineWidth',3,'LineStyle',':','color',"#F5A5A7")
errorbar(T_flex.Strain_0*10,T_flex.Stifness_0,T_flex.Stifness_1-T_flex.Stifness_0,T_flex.Stifness_2-T_flex.Stifness_0,'LineWidth',3,'LineStyle','none','color',map(1,:))


% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 2.5;
height = 1.75;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
ylim([0,65])
xlim([-2.5,22])


% export fig
if export_fig
    exportgraphics(gcf,'../figures/instron/flex-shaft.png','Resolution',300*fig_s)
end

%% Simple printed force

fig = figure(2); clf; hold on;

shadedErrorBar(T_spf.Displacment_0,T_spf.Force_0,[T_spf.Force_2-T_spf.Force_0,-(T_spf.Force_1-T_spf.Force_0)],'lineProps',{'Color',map(3,:),'lineWidth',3,'MarkerSize',10,"DisplayName",""})

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
xticks([0,2,4,6,8])
yticks([0,2.5,5])
xlim([0,8])

% export fig
if export_fig
    exportgraphics(gcf,'../figures/instron/spf.png','Resolution',300*fig_s)
end

%% Simple printed torque

fig = figure(3); clf; hold on;

shadedErrorBar(T_spt_a.Rotation_0,T_spt_a.Torque_0,[T_spt_a.Torque_2-T_spt_a.Torque_0,-(T_spt_a.Torque_1-T_spt_a.Torque_0)],'lineProps',{'Color',map(2,:),'lineWidth',3,'MarkerSize',10,"DisplayName",""})
shadedErrorBar(T_spt_p.Rotation_0,T_spt_p.Torque_0,[T_spt_p.Torque_2-T_spt_p.Torque_0,-(T_spt_p.Torque_1-T_spt_p.Torque_0)],'lineProps',{'Color',map(1,:),'lineWidth',3,'MarkerSize',10,"DisplayName",""})

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
xticks([0,0.15,.3])
yticks([0,25,50])
xlim([0,.35])


% legend formatting
lg = legend('interpreter','latex','Location','northwest');
lg.FontSize = legend_font_size;
set(lg,'Box','off')
lg.ItemTokenSize(1) = 20;

% export fig
if export_fig
    exportgraphics(gcf,'../figures/instron/spt.png','Resolution',300*fig_s)
end


%% Simple steel force

fig = figure(4); clf; hold on;

shadedErrorBar(T_ssf.Displacment_0,T_ssf.Force_0,[T_ssf.Force_2-T_ssf.Force_0,-(T_ssf.Force_1-T_ssf.Force_0)],'lineProps',{'Color',map(3,:),'lineWidth',3,'MarkerSize',10,"DisplayName",""})

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
xticks([0,5,10,15])
xlim([0,17.5])

% export fig
if export_fig
    exportgraphics(gcf,'../figures/instron/ssf.png','Resolution',300*fig_s)
end


%% Simple steel torque

fig = figure(5); clf; hold on;

shadedErrorBar(T_sst_a.Rotation_0,T_sst_a.Torque_0,[T_sst_a.Torque_2-T_sst_a.Torque_0,-(T_sst_a.Torque_1-T_sst_a.Torque_0)],'lineProps',{'Color',map(2,:),'lineWidth',3,'MarkerSize',10,"DisplayName",""})
shadedErrorBar(T_sst_p.Rotation_0,T_sst_p.Torque_0,[T_sst_p.Torque_2-T_sst_p.Torque_0,-(T_sst_p.Torque_1-T_sst_p.Torque_0)],'lineProps',{'Color',map(1,:),'lineWidth',3,'MarkerSize',10,"DisplayName",""})

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
xticks([0,0.15,.3])
yticks([0,25,50])
xlim([0,.35])


% legend formatting
lg = legend('interpreter','latex','Location','northwest');
lg.FontSize = legend_font_size;
set(lg,'Box','off')
lg.ItemTokenSize(1) = 20;

% export fig
if export_fig
    exportgraphics(gcf,'../figures/instron/sst.png','Resolution',300*fig_s)
end

%% Truss steel force

fig = figure(6); clf; hold on;

shadedErrorBar(T_tsf.Displacment_0,T_tsf.Force_0,[T_tsf.Force_2-T_tsf.Force_0,-(T_tsf.Force_1-T_tsf.Force_0)],'lineProps',{'Color',map(3,:),'lineWidth',3,'MarkerSize',10,"DisplayName",""})

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
xticks([0,5,10,15])
yticks([0,5,10])
xlim([0,17])


% export fig
if export_fig
    exportgraphics(gcf,'../figures/instron/tsf.png','Resolution',300*fig_s)
end

%% Truss steel torque

fig = figure(7); clf; hold on;

shadedErrorBar(T_tst_a.Rotation_0,T_tst_a.Torque_0,[T_tst_a.Torque_2-T_tst_a.Torque_0,-(T_tst_a.Torque_1-T_tst_a.Torque_0)],'lineProps',{'Color',map(2,:),'lineWidth',3,'MarkerSize',10,"DisplayName",""})
shadedErrorBar(T_tst_p.Rotation_0,T_tst_p.Torque_0,[T_tst_p.Torque_2-T_tst_p.Torque_0,-(T_tst_p.Torque_1-T_tst_p.Torque_0)],'lineProps',{'Color',map(1,:),'lineWidth',3,'MarkerSize',10,"DisplayName",""})

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
xticks([0,0.15,.3])
yticks([0,50,100])
xlim([0,.35])

% legend formatting
lg = legend('interpreter','latex','Location','northwest');
lg.FontSize = legend_font_size;
set(lg,'Box','off')
lg.ItemTokenSize(1) = 20;

% export fig
if export_fig
    exportgraphics(gcf,'../figures/instron/tst.png','Resolution',300*fig_s)
end
