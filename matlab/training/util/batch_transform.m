function T_new = batch_transform(T1,T2)

    % Change of coordinates so (0,0,0) matches
    T2.x_end_avg = (T2.x_end_avg - T2.x_end_avg(1)) + T2.x_end_avg(1);
    T2.y_end_avg = (T2.y_end_avg - T2.y_end_avg(1)) + T2.y_end_avg(1);
    T2.z_end_avg = (T2.z_end_avg - T2.z_end_avg(1)) + T2.z_end_avg(1);
    
    q1 = [T1.qw_end_avg(1),T1.qx_end_avg(1),T1.qy_end_avg(1),T1.qz_end_avg(1)];
    p1 = [T1.x_end_avg(1),T1.y_end_avg(1),T1.z_end_avg(1)];

    q2 = [T2.qw_end_avg(1),T2.qx_end_avg(1),T2.qy_end_avg(1),T2.qz_end_avg(1)];
    p2 = [T2.x_end_avg(1),T2.y_end_avg(1),T2.z_end_avg(1)];
    
    v_names = T1.Properties.VariableNames;
    T_new = array2table(zeros([size(T2,1),7]),'VariableNames',v_names);

    

    for i = 1:size(T2,1)
        qi = [T2.qw_end_avg(i),T2.qx_end_avg(i),T2.qy_end_avg(i),T2.qz_end_avg(i)];
        pi = [T2.x_end_avg(i),T2.y_end_avg(i),T2.z_end_avg(i)];
        [qi_new,pi_new] = coordinate_transform(q1,p1,q2,p2,qi,pi);
        qi_new = [qi_new(2:4),qi_new(1)];
        T_new(i,:) = num2cell([pi_new.',qi_new]);
    end

end