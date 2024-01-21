function motor_pos = delta2pos(d_end,d_body,dL)
% Input: 
%       Array of motor deltas 'd0_end', 'd1_end', 'd2_end', 'd0_body',
%       'd1_body', 'd2_body', 'dL'

% Output: 
%       motor_pos - Global positions of the 9 motor which are passed to set_servo_position_auxarm.m

% Load compression values
home_comp = load("home_comp.mat").home_comp;
comp_ratio = load("comp_ratio.mat");

n = size(dL,1);

% Normalize to prevent length change
d_end = d_end-min(d_end,[],2);
d_body = d_body-min(d_body,[],2);

motor_pos = [d_end(:,1),d_body(:,1).*comp_ratio.c_1,d_body(:,1),...
             d_end(:,2),d_body(:,2).*comp_ratio.c_1,d_body(:,2),...
             d_end(:,3),d_body(:,3).*comp_ratio.c_1,d_body(:,3),...
             zeros(n,1),zeros(n,1),zeros(n,1)] + home_comp + dL;

motor_pos(:,10:end) = zeros(n,3);
end