% Control - 1 DoF Robotic Arm
%{
Departamento de Engenharia Mecânica
Escola de Engenharia de São Carlos - Universidade de São Paulo
Área de Concentração: Dinâmica e Mecatrônica
%}

clc;
clear all;
close all;
addpath('..\..\shared-functions')

% Play animation?
play = 1;
isBackdrivable = 1;

% Model parameters
[t, dt] = SimulationTime(0,6,5e-3);
run('Variables_1DoF.m')

% Desired trajectory
q_d = 1*sin(pi*t)';

for i = 1 : length(t)
    [x_d, y_d] = L_forward(L,q_d(i),dof);
    X_d(:, i) = [x_d ; y_d];
end

% Environment
K_env = diag([0, 5e2]);
B_env = diag([0, 1]);

% Obstacle
wall_x = 10*L;              %very far away wall, aka no restrictions
wall_y = 0.2;
wall = [wall_x; wall_y];

% Impedance Control
K_d = diag([0, 1e3]);
B_d = diag([0, 1e-1]);

% PID Force Control Gains
k_p = 100;
k_d = 10;
k_i = 0;

% Simulation
for i = 1 : length(t) - 1
    
    %Jacobian calculation
    Jacobian = JacobianMatrix(L,q(i),dof);
            
    %End-effector variables
    [x, y] = L_forward(L,q(i),dof);
    X(:, i) = [x; y];
    
    dX(:, i) = Jacobian*dq(i,:);
    dx = dX(1,:);
    dy = dX(2,:);
    
    %Force reference based on error (Impedance Control Law)
    F_r(:, i) = K_d*(X_d(:, i) - X(:, i)) - B_d*(dX(:, i));
    
    % Environment detection
    if(x > wall_x || y > wall_y)
        F_ext(:, i) = K_env*(X(:, i) - wall) + K_env*dX(:, i);
    end
    
    % Force Error
    F_error(:, i) = F_r(:, i) + F_d(:, i) - F_ext(:, i);
    
    if i ~= 1
       F_int_error(:,i) = F_int_error(:,i - 1) + (F_error(:,i) + F_error(:,i - 1))*dt/2;
       dF_error(:,i) = ( F_error(:,i) - F_error(:,i-1) ) / dt;
    end
    
    % Control signal
    tau_a(i) = Jacobian'*(k_p * F_error(:,i) + k_d * dF_error(:,i) + k_i * F_int_error(:,i)) + (L/2)*m*g*cos(q(i));
    tau_a(i) = Saturation(tau_a(i), torque_max, torque_min);

    % Outputs
    ddq(i) = (1/J)*(tau_a(i) - (L/2)*m*g*cos(q(i)) - B*dq(i) - K*q(i));
    dq(i + 1) = dq(i) + ddq(i)*dt;
    q(i + 1) = q(i) + dq(i)*dt + 0.5*ddq(i)*dt*dt;
end

% Graphic Simulation
if(play)
    Animation(q, q_d, L, t, offset, wall_y, dof);
end