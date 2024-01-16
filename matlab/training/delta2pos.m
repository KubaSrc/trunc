function motor_pos = delta2pos(T,home_comp)
% Input: 
%       T - table of motor deltas 'de_theta', 'de_phi', 'dtm_theta', 'dtm_phi', 'dl'
%       home_comp - compression values for each motor

% Output: 
%       motor_pos - Global positions of the 9 motor which are passed to set_servo_position_auxarm.m

% Load compression values
home_comp = load("home_comp.mat").home_comp;

% Trinagular coordinates
dz = -65;
p1 = [-65*sin(pi/3); -65*cos(pi/3); dz; 1];
p2 = [65; 0; dz; 1];
p3 = [65*sin(pi/3); -65*cos(pi/3); dz; 1];
P = [p1, p2, p3];

% Initialize matrices to store results
num_thetas = size(T.de_theta, 1);
P_e = zeros(4, 3, num_thetas);
P_tm = zeros(4, 3, num_thetas);

% Apply rotations for each theta and phi value
for i = 1:num_thetas
    rx_e= [1, 0, 0, 0; 
                0, cos(T.de_theta(i)), -sin(T.de_theta(i)), 0; 
                0, sin(T.de_theta(i)), cos(T.de_theta(i)), 0; 
                0, 0, 0, 1];
    ry_e = [cos(T.de_phi(i)), 0, sin(T.de_phi(i)), 0; 
                0, 1, 0, 0; 
                -sin(T.de_phi(i)), 0, cos(T.de_phi(i)), 0; 
                0, 0, 0, 1];

    rx_tm = [1, 0, 0, 0; 
                0, cos(T.dtm_theta(i)), -sin(T.dtm_theta(i)), 0; 
                0, sin(T.dtm_theta(i)), cos(T.dtm_theta(i)), 0; 
                0, 0, 0, 1];
    ry_tm = [cos(T.dtm_phi(i)), 0, sin(T.dtm_phi(i)), 0; 
                0, 1, 0, 0; 
                -sin(T.dtm_phi(i)), 0, cos(T.dtm_phi(i)), 0; 
                0, 0, 0, 1];

    P_e(:,:,i) = rx_e * ry_e * P;
    P_tm(:,:,i) = rx_tm * ry_tm * P; % Modify as needed
end

% Length extension or contraction using COM for end triangle
dP_e = vecnorm(P_e - P, 2, 1);
P_e_com = mean(P_e,2);
dP_e_signed = dP_e.*(-(P_e(3,:,:) < P_e_com(3,:,:))+(P_e(3,:,:) > P_e_com(3,:,:)));
dP_e_signed = [reshape(dP_e_signed(:,1,:),[],1),reshape(dP_e_signed(:,2,:),[],1),reshape(dP_e_signed(:,3,:),[],1)];

% Same for top and middle
dP_tm = vecnorm(P_tm - P, 2, 1);
P_tm_com = mean(P_tm,2);
dP_tm_signed = dP_tm.*(-(P_tm(3,:,:) < P_tm_com(3,:,:))+(P_tm(3,:,:) > P_tm_com(3,:,:)));
dP_tm_signed = [reshape(dP_tm_signed(:,1,:),[],1),reshape(dP_tm_signed(:,2,:),[],1),reshape(dP_tm_signed(:,3,:),[],1)];

% Calculate motor positions from rotations, length changes
motor_pos = [dP_e_signed, dP_tm_signed, dP_tm_signed,...
            zeros(size(T,1),3)] + T.dl.*[ones(1,9),zeros(1,3)] + home_comp;
end