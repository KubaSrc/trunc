%% Global setup

close all; clear all; clc
warning('off','all')

% fig sizes and scale factor
fig_w = 300; fig_h = 300; fig_s = 3;

% fonts
ax_font_size = 6*fig_s;
legend_font_size = 4*fig_s;
set(0,'DefaultTextFontname', 'CMU Sans Serif' )
set(0,'DefaultAxesFontName', 'CMU Sans Serif' )

% colors
map = brewermap(9,'Set1');

% save path
export_fig = true;

%% TRUNC

beta = 0:5:45;
files = strcat(string(0:5:45),repmat("deg.tdms",1,10));

input_data = load_data("./characterization/Efficiency Tests/Data/TRUNC/Input/",files);
output_data = load_data("./characterization/Efficiency Tests/Data/TRUNC/Output/",files);

power_in = zeros(size(beta));
power_out = zeros(size(beta));
window = 800;
offset = 300;
threshold = 0.05;

for i = 1:length(beta)

    % Find input power
    data_input = input_data{i};
    t_in = table2array(data_input(:,1));
    r_in = table2array(data_input(:,2));
    p_in = table2array(data_input(:,3));
    index = find(r_in > 29 & t_in > 0.225, 1);
    p_in_mean = mean(p_in(index+offset:index+window+offset,:),1);
    power_in(i) = p_in_mean;

    % Find output power
    data_output = output_data{i};
    t_out = table2array(data_output(:,1));
    r_out = table2array(data_output(:,2));
    p_out = table2array(data_output(:,3));
    index = find(r_out > 29 & t_out > 0.225, 1);
    p_out_mean = mean(p_out(index+offset:index+window+offset,:),1);
    power_out(i) = p_out_mean;

end

fig = figure(1); clf; hold on; grid on; box on;

plot(beta,100.*power_out./power_in,'o-.','markerFaceColor',"k",'color',"k",'lineWidth',2)

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
ylim([0,100]);
xlim([0,45])

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 2.25;
height = 1.75;
set(fig, 'Position', [0, 0, width*2.5, height*2.5]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);

%% Rubber

beta = 0:5:45;
files = strcat(string(0:5:45),repmat("deg.tdms",1,10));

input_data = load_data("./characterization/Efficiency Tests/Data/Rubber/Input/",files);
output_data = load_data("./characterization/Efficiency Tests/Data/Rubber/Output/",files);

power_in = zeros(size(beta));
power_out = zeros(size(beta));
window = 800;
offset = 300;
threshold = 29;

for i = 1:length(beta)

    % Find input power
    data_input = input_data{i};
    p_in = table2array(data_input(:,3));
    r_in = table2array(data_input(:,2));
    index = find(r_in > threshold, 1);
    p_in_mean = mean(p_in(index+offset:index+window+offset,:),1);
    power_in(i) = p_in_mean;

    % Find output power
    data_output = output_data{i};
    p_out = table2array(data_output(:,3));
    r_out = table2array(data_output(:,2));
    index = find(r_out > threshold, 1);
    p_out_mean = mean(p_out(index+offset:index+window+offset,:),1);
    power_out(i) = p_out_mean;

end


plot(beta,100.*power_out./power_in,'o-.','markerFaceColor',map(2,:),'color',map(2,:),'lineWidth',2)


%% Steel

beta = 0:5:10;
files = strcat(string(0:5:10),repmat("deg.tdms",1,3));

input_data = load_data("./characterization/Efficiency Tests/Data/Steel/Input/",files);
output_data = load_data("./characterization/Efficiency Tests/Data/Steel/Output/",files);

power_in = zeros(size(beta));
power_out = zeros(size(beta));
window = 800;
offset = 300;
threshold = 28;

for i = 1:length(beta)

    % Find input power
    data_input = input_data{i};
    p_in = table2array(data_input(:,3));
    r_in = table2array(data_input(:,2));
    index = find(r_in > threshold, 1);
    p_in_mean = mean(p_in(index+offset:index+window+offset,:),1);
    power_in(i) = p_in_mean;

    % Find output power
    data_output = output_data{i};
    p_out = table2array(data_output(:,3));
    r_out = table2array(data_output(:,2));
    index = find(r_out > threshold, 1);
    p_out_mean = mean(p_out(index+offset:index+window+offset,:),1);
    power_out(i) = p_out_mean;

end


plot(beta,100.*power_out./power_in,'o-.','markerFaceColor',map(1,:),'color',map(1,:),'lineWidth',2)



% export fig
if export_fig
    exportgraphics(gcf,'../figures/efficiency/eta-plot.png','Resolution',300*fig_s)
end



%% Helper functions
function [allData] = load_data(folder,files)

% Initialize a cell array to store the data from each file
allData = {};

% Loop through each TDMS file
for i = 1:length(files)
    
    % Full path to the TDMS file
    tdmsFile = folder + files(i);
    
    % Read the TDMS file (assuming you're using the built-in tdmsread function)
    data = tdmsread(tdmsFile);
    
    allData{i} = data{1};
end

end