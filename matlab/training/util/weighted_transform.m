function T_prime = weighted_transform(T_w)
    v_names = T_w.Properties.VariableNames;
    T_prime = array2table(zeros([size(T_w,1),7]),'VariableNames',v_names);

    % Transform positions
    T_prime.x_end_avg = T_w.x_end_avg;
    T_prime.y_end_avg = -T_w.z_end_avg;
    T_prime.z_end_avg = T_w.y_end_avg;


    % Assign transformed quaternions to T_prime
    T_prime.qw_end_avg = T_w.qw_end_avg;
    T_prime.qx_end_avg = T_w.qx_end_avg;
    T_prime.qy_end_avg = -T_w.qz_end_avg;
    T_prime.qz_end_avg = T_w.qy_end_avg;
end