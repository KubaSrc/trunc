function [qi_new,pi_new] = coordinate_transform(q1,p1,q2,p2,qi,pi)

q2 = q2 / norm(q2);
q2_conj = [q2(1), -q2(2), -q2(3), -q2(4)];

% Quaternion multiplication (q1_conj * q2)
q_rel = quatmultiply(q2_conj, q1);

% Convert the relative quaternion to a rotation matrix
R_rel = quat2rotm(q_rel);
R_2 = quat2rotm(q2);

qi = qi / norm(qi);

R_i = quat2rotm(qi);

% Find new orientation
qi_new = rotm2quat(R_rel*R_i);

Ri_new = quat2rotm(qi_new);

d_q = p1-p2;

pi_new = pi.' + Ri_new*d_q.';

end