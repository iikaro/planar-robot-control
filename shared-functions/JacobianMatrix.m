function [J] = JacobianMatrix(l,q,dof)
% JACOBIANMATRIX The Jacobian of 2 rows and N columns of a planar robot
% with N degrees-of-freedom.
% 
% JACOBIANMATRIX(L, Q, DOF) returns a matrix of 2-rows and N-columns. L is
% the column (or row) vector with the link length of the robot, Q is the
% column (or row) vector with the instantaneous joint displacement of each
% robot join. DOF is the degrees-of-freedom of the robot.
%
% Example: 2-DoF robot.
% JacobianMatrix([1;1], [pi;pi/2], 2) returns:
%
%     1.0000    1.0000
%    -1.0000   -0.0000
if dof == 1
    J = [-l(1)*sin(q(1));
        l(1)*cos(q(1))];
    
elseif dof == 2
    J11 = -(l(1)*sin(q(1)) + l(2)*sin(q(1) + q(2)));
    J12 = - l(2)*sin(q(1) + q(2));
    J21 = l(1)*cos(q(1)) + l(2)*cos(q(1) + q(2));
    J22 = l(2)*cos(q(1) + q(2));
    
    J = [J11, J12;
        J21, J22];

elseif dof == 3
    J11 = -(l(1)*sin(q(1)) + l(2)*sin(q(1) + q(2)) + l(3)*sin(q(1) + q(2) + q(3)));
    J12 = - l(2)*sin(q(1) + q(2)) - l(3)*sin(q(1) + q(2) + q(3));
    J21 = l(1)*cos(q(1)) + l(2)*cos(q(1) + q(2)) + l(3)*cos(q(1) + q(2) + q(3));
    J22 = l(2)*cos(q(1) + q(2)) + l(3)*cos(q(1) + q(2) + q(3));
    J31 = - l(3)*sin(q(1) + q(2) + q(3));
    J32 = l(3)*cos(q(1) + q(2) + q(3));
    
    J = [J11, J12, J13;
        J21, J22, J32];
end
    
    
end