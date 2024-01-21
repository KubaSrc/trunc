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

% Maximum cable deltas and maximum length change
d_max = 40;
L_max = 40;

% Random sample size
rng(8);
n = 15000;

% End effector
d_end = -sqrt(normrnd(0,d_max/3,[n,3]).^2);

% Body
d_body = -sqrt(normrnd(0,d_max/3,[n,3]).^2);

% Length
dL = -sqrt(normrnd(0,L_max/3,[n,1]).^2);

motor_deltas = [d_end, d_body, dL];
%% Calculate absolute motor position

motor_pos = delta2pos(d_end,d_body,dL);

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