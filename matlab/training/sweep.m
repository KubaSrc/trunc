%% Global setup

close all; clear all; clc
warning('off','all')
% Fonts
ax_font_size = 30; label_font_size = 36; 
legend_font_size = 28; title_font_size = 34;
% Figures
fig_w = 750; fig_h = 600; fig_s = 1.6;
set(0,'DefaultTextFontname', 'CMU Sans Serif' )
set(0,'DefaultAxesFontName', 'CMU Sans Serif' )
map = brewermap(9,'Set1');

%% Generate random relative motor positions

% Define maximum tilt and length changes
theta_max = deg2rad(15);
phi_max = deg2rad(15);
l_max = 20;

% Random sample size
rng(8);
n = 15000;

% End rotation 
de_theta = normrnd(0,theta_max/3,[n,1]);
de_phi= normrnd(0,phi_max/3,[n,1]);

% Top middle rotation
dtm_theta = normrnd(0,theta_max/3,[n,1]);
dtm_phi = normrnd(0,phi_max/3,[n,1]);

% Length change
dl = normrnd(0,l_max/3,[n,1]);

motor_deltas = [de_theta,de_phi,dtm_theta,dtm_phi,dl];
T = array2table(motor_deltas);
T.Properties.VariableNames(1:5) = {'de_theta','de_phi','dtm_theta','dtm_phi','dl'};
writetable(T,['motor_delta_sweep.csv'])

%% Calculate absolute motor position

md = T;

motor_pos = delta2pos(md);
motor_deltas_fast = zeros(size(motor_deltas));

save("./motor_pos_repeat.mat","motor_pos")

%% Greedy solution to TSP

motor_pos_fast = zeros(size(motor_pos));
motor_pos_fast(1,:) = motor_pos(1,:);

% Keep track of which answers to exclude
exclude = true(size(motor_pos,1),1);
exclude(1) = false;

for i = 1:size(motor_pos_fast,1)-1
    % Keep track of original indices
    original_idx = find(exclude);
    
    motor_trim = motor_pos(exclude,:);
    idx = knnsearch(motor_trim,motor_pos_fast(i,:));
    
    % Update indices
    original_idx_selected = original_idx(idx);
    
    motor_pos_fast(i+1,:) = motor_pos(original_idx_selected,:);
    motor_deltas_fast(i+1,:) = motor_deltas(original_idx_selected,:);
    exclude(original_idx_selected) = false;
end

fast_diff = sqrt(sum(diff(motor_pos_fast).^2,2));
regular_diff = sqrt(sum(diff(motor_pos).^2,2));

mean(fast_diff)
mean(regular_diff)

fig = figure(1); clf; hold on;
set(gcf,'color','w');
set(gcf,'position',[0,0,fig_w*fig_s,fig_h*fig_s])
set(gca,'fontsize',ax_font_size);
ax = gca;
grid on; grid minor;
set(get(fig,'CurrentAxes'),'GridAlpha',1,'MinorGridAlpha',0.7);
plot(fast_diff,'lineWidth',3,'Color',map(1,:));
plot(regular_diff,'lineWidth',3,'Color',map(2,:));
legend(["Greedy KNN","random"])
xlabel("Data point index",fontSize=label_font_size)
ylabel("Total motor displacment",fontSize=label_font_size)

% Save output
save("./motor_pos_fast.mat","motor_pos_fast")
writematrix(motor_pos_fast,"./motor_pos_fast.csv")
writematrix(motor_deltas_fast,"./motor_deltas_fast.csv")

%% Sample for repetability

% Greedy solution to TSP
[motor_pos_repeat,sample_idx] = datasample(motor_pos,100);
motor_pos_repeat_fast = zeros(size(motor_pos_repeat));
motor_pos_repeat_fast(1,:) = motor_pos_repeat(1,:);

md_repeat = motor_deltas(sample_idx,:);
md_repeat_fast = zeros(size(md_repeat));
md_repeat_fast(1,:) = md_repeat_fast(1,:);


% Keep track of which answers to exclude
exclude = true(size(motor_pos_repeat,1),1);
exclude(1) = false;

for i = 1:size(motor_pos_repeat_fast,1)-1
    % Keep track of original indices
    original_idx = find(exclude);
    
    motor_trim = motor_pos_repeat(exclude,:);
    idx = knnsearch(motor_trim,motor_pos_repeat_fast(i,:));
    
    % Update indices
    original_idx_selected = original_idx(idx);
    
    motor_pos_repeat_fast(i+1,:) = motor_pos_repeat(original_idx_selected,:);
    md_repeat_fast(i+1,:) = md_repeat(original_idx_selected,:);
    exclude(original_idx_selected) = false;
end

save("./motor_pos_repeat_fast.mat","motor_pos_repeat_fast")
writematrix(md_repeat_fast,"./md_repeat_fast.csv")