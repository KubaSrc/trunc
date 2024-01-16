% dt_r1,dt_r2,dt_l,dm_r1,dm_r2,dm_l,de_r1,de_r2,de_l
clear all; clc;

% Define maximum tilt and length changes
l_max_t = 45;
l_max_m = 40;
l_max_e = 40;
l_max = 13;

rs = 0.1; % scale rotation coupling

rng(1);
n = 30000;

% Top stage
dt_13 = 2*(rand([n-1,1])-.5)*r_max_t;
dt_24 = 2*(rand([n-1,1])-.5)*r_max_t;
dt_l = 2*(rand([n-1,1])-.5)*l_max_t;

% Middle stage
dm_13 = 2*(rand([n-1,1])-.5)*r_max_m;
dm_24 = 2*(rand([n-1,1])-.5)*r_max_m;
dm_l = 2*(rand([n-1,1])-.5)*l_max_m;

% End stage
de_13 = 2*(rand([n-1,1])-.5)*r_max_e;
de_24 = 2*(rand([n-1,1])-.5)*r_max_e;
de_l = 2*(rand([n-1,1])-.5)*l_max_e;

motor_deltas = [zeros([1,9]);dt_13,dt_24,dt_l,dm_13,dm_24,dm_l,de_13,de_24,de_l];
T = array2table(motor_deltas);
T.Properties.VariableNames(1:9) = {'dt_13','dt_24','dt_l','dm_13','dm_24','dm_l','de_13','de_24','de_l'};
writetable(T,'motor_deltas.csv')

% Define home position with compression values
t_c = -20; m_c = -18; e_c = -12;
t_home = [-2,-3,-12,-4]+t_c;
m_home = [0,-12,-5,0]+t_c+m_c;
e_home = [-3,-10,-3,-24]+t_c+m_c+e_c;
home_position =  [t_home(1),m_home(1),e_home(1),t_home(2),m_home(2),e_home(2),t_home(3),m_home(3),e_home(3),t_home(4),m_home(4),e_home(4)];

% Calculate positions from motor deltas
md = T;
motor_pos = [t_home(1)+md.dt_l+md.dt_13,m_home(1)+md.dt_l+md.dm_l+md.dm_13,e_home(1)+md.dt_l+md.dm_l+md.de_l+md.de_13,...
            t_home(2)+md.dt_l+md.dt_24,m_home(2)+md.dt_l+md.dm_l+md.dm_24,e_home(2)+md.dt_l+md.dm_l+md.de_l+md.de_24,...
            t_home(3)+md.dt_l-md.dt_13,m_home(3)+md.dt_l+md.dm_l-md.dm_13,e_home(3)+md.dt_l+md.dm_l+md.de_l-md.de_13,...
            t_home(4)+md.dt_l-md.dt_24,m_home(4)+md.dt_l+md.dm_l-md.dm_24,e_home(4)+md.dt_l+md.dm_l+md.de_l-md.de_24];
writematrix(motor_pos,'motor_raw_input.csv')