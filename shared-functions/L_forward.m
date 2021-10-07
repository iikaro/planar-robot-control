function [x, y, theta] = L_forward(l, q, dof)
% L_FOWARD The forward kinematics of a planar robot.
%
% [X, Y, THETA] = L_FOWARD(L, Q, DOF) Returns the instantaneous X, Y and
% end-effector orientation THETA of a planar robot with DOF
% degrees-of-freedom, given the robot link's lengths L, instantaneous joint
% displacement Q, and number of degrees-of-freedom DOF.

x = 0;
y = 0;
theta = 0;

if dof == 1
    x = l*cos(q(1,:));
    y = l*sin(q(1,:));
    theta = q;
elseif dof == 2
    x = l(1)*cos(q(1,:)) + l(2)*cos(q(1,:) + q(2,:));
    y = l(1)*sin(q(1,:)) + l(2)*sin(q(1,:) + q(2,:));
    theta = q(1,:) + q(2,:);
elseif dof == 3
    x = l(1)*cos(q(1,:)) + l(2)*cos(q(1,:) + q(2,:)) + l(3)*cos(q(1,:) + q(2,:) + q(3,:));
    y = l(1)*sin(q(1,:)) + l(2)*sin(q(1,:) + q(2,:)) + l(3)*sin(q(1,:) + q(2,:) + q(3,:));
    theta = q(1,:) + q(2,:) + q(3,:);
end
end