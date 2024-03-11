close all; clear all; clc;

T = readtable('./data/2024_02_19_21_08_57/positions_norm_full.csv');

ref = load('./inference/motherboard_trajectory.mat').wp;

actual = readtable('./experiments/motherboard_2024_02_28_18_00_28/positions.csv');

actual_new = batch_transform(T,actual);

% Tile 2: X-Z
figure(1); clf;
hold on; grid on; axis equal;
set(gcf, 'Color', 'white');

% Replotting for the second tile with X-Z view
plot3(actual_new(:,1).*1000,actual_new(:,2).*1000,actual_new(:,3).*1000,'-','MarkerSize',8,'LineWidth',2,'DisplayName','DNN');
plot3(ref(:,1),ref(:,2),ref(:,3),'-','Color','red','LineWidth',2,'DisplayName','actual');


%% Motherboard demo analysis

T = readtable('./demo/2024_02_19_21_08_57/MOTHERBOARD-2024-02-29 12.37.45 PM');

%% Lightbulb demo analysis

T = readtable('./demo/2024_02_19_21_08_57/LIGHTBULB-2024-03-06 11.55.52 AM');

%% Valve demo analysis

% TO DO

%% Helper functions
% Applies transformation to T2 based on origin pose described by T1
function w_new = batch_transform(T1,T2)
    
    q1 = [T1.qw_end_avg(1),T1.qx_end_avg(1),T1.qy_end_avg(1),T1.qz_end_avg(1)];
    p1 = [T1.x_end_avg(1),T1.y_end_avg(1),T1.z_end_avg(1)];
    

    q2 = [T2.qw_end_avg(1),T2.qx_end_avg(1),T2.qy_end_avg(1),T2.qz_end_avg(1)];
    p2 = [T2.x_end_avg(1),T2.y_end_avg(1),T2.z_end_avg(1)];
    

    w_new = zeros([size(T2,1),7]);

    for i = 1:size(T2,1)
        qi = [T2.qw_end_avg(i),T2.qx_end_avg(i),T2.qy_end_avg(i),T2.qz_end_avg(i)];
        pi = [T2.x_end_avg(i),T2.y_end_avg(i),T2.z_end_avg(i)];
        [qi_new,pi_new] = coordinate_transform(q1,p1,q2,p2,qi,pi);
        w_new(i,1:3) = pi_new.';
        w_new(i,4:end) = [qi_new(2:4),qi_new(1)];
    end

end