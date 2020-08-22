function [F_int] = OnCollisionStay(x, dx, x_w, dx_w, K_fs, B_fs, wall, SEA, q_m, dq_m, q_d, l)
if wall == 0, i = 1; j = 2;             %horizontal wall
elseif wall == 1, i = 2; j = 1; end     %vertical wall

% if (SEA)
%     F_int(i,1) = -(q_d(j) - q_m(j))*K_fs/l(2) + B_fs*dq_m(j)/l(2);
%     if(F_int(i,1) < 0), F_int(i,1) = 0; end
%     F_int(j,1) = 0;
%     return
% end

if x_w(1) > 0;
    F_int(i,1) = K_fs*(x(j) - x_w) + B_fs*dx(j);
    if(F_int(i,1) < 0), F_int(i,1) = 0; end
    F_int(j,1) = 0;
    return
elseif x_w(1) < 0;
    F_int(i,1) = K_fs*(x(j) - x_w) - B_fs*dx(i);
    if(F_int(i,1) < 0), F_int(i,1) = 0; end
    F_int(j,1) = 0;
end
end