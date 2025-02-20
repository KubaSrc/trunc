function [qi_new] = quaternion_transform(q1, q2, qi)
    % Normalize q2 and compute its conjugate (inverse for unit quaternions)
    q2 = q2 / norm(q2);
    q2_conj = [q2(1), -q2(2), -q2(3), -q2(4)];
    
    % Compute the relative quaternion correctly: q_rel = q1 * inv(q2)
    q_rel = quatmultiply(q1, q2_conj);
    
    % Normalize the input orientation quaternion
    qi = qi / norm(qi);
    
    % Apply the relative rotation: qi_new = q_rel * qi
    % This can be done directly with quaternion multiplication:
    qi_new = quatmultiply(q_rel, qi);
end