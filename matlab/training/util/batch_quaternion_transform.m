function T_new = batch_quaternion_transform(T_old,q2)

    q1 = [1,0,0,0];

    if nargin < 2
        q2 = [T_old.qw_end_avg(1),T_old.qx_end_avg(1),T_old.qy_end_avg(1),T_old.qz_end_avg(1)];
    else
        disp(q2)
    end
       
    T_new = T_old;

    for i = 1:size(T_old,1)
        qi = [T_old.qw_end_avg(i),T_old.qx_end_avg(i),T_old.qy_end_avg(i),T_old.qz_end_avg(i)];
        [qi_new] = quaternion_transform(q1,q2,qi);
        T_new.qw_end_avg(i) = qi_new(1);
        T_new.qx_end_avg(i) = qi_new(2);
        T_new.qy_end_avg(i) = qi_new(3);
        T_new.qz_end_avg(i) = qi_new(4);
    end

end