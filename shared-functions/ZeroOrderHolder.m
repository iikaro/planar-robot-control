%% Zero Order Holder
F_ext(:,i) = F_ext(:, i -1);
T_ext(:, i) = T_ext(:, i - 1);
T_error_out(:, i) = T_error_out(:, i - 1);
T_error_in(:,i) = T_error_in(:,i-1);
T_error_in_int(:,i) = T_error_in_int(:,i-1);
dT_error_in(:,i) = dT_error_in(:,i-1);
T_a(: , i) = T_a(:, i - 1);
T_r(: , i) = T_r(:, i - 1);
q_r(:,i) =  q_r(:,i - 1);
T_p(:,i) = T_p(:,i - 1);
dq_r(:,i) = dq_r(:,i - 1);
q_p(:,i) = q_p(:,i-1);
dq_p(:,i) = dq_p(:,i-1);