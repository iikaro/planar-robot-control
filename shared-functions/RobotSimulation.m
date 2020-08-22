function [ddq, dq_next, q_next] = RobotSimulation(H, G, C, F, Jacobian, torque, F_ext, dq, q, dt, isBackdrivable, dof)

if dof == 1
    ddq = (1/H)*(torque - G - F*dq - isBackdrivable*Jacobian'*F_ext);
    dq_next = dq + ddq*dt;
    q_next = q + dq*dt + 0.5*ddq*dt*dt;
    
elseif dof == 2
    ddq = H\(T_d(:, i) + T_a(:,i) - C*dq_m(:, i) - G - F*dq_m(:, i)  - tau_ext(:,i));
    dq_next = dq + ddq*dt;
    q_next = q + dq*dt + 0.5*ddq*dt*dt;
end
end