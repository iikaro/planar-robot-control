% Control - 2 DoF Robotic Arm
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
play = 0;
isBackdrivable = 0;
isImpedance = 1;
isTaskSpace = 0;

% Model parameters
[t, dt] = SimulationTime(0,10,1e-4);
dt_c = 5e-3;

% Variables
run('TwoDOF_Variables.m')

% Desired trajectory
offset = [0; -pi/2];
A = 60*pi/180;
w = 1*pi;

% First joint
q_d(1,:) = 0*A*sin(w*t) + offset(1);
dq_d(1,:) = 0*w*A*cos(w*t);

% Second joint
q_d(2,:) = A*sin(w*t) + offset(2);
dq_d(2,:) = w*A*cos(w*t);

% Desired trajectory
for i = 1 : length(t)
    if t(i) >= 2.5, q_d(2,i) = q_d(2,i-1); end
end

x_d = l(1)*cos(q_d(1,:)) + l(2)*cos(q_d(1,:) + q_d(2,:));
y_d = l(1)*sin(q_d(1,:)) + l(2)*sin(q_d(1,:) + q_d(2,:));

% Initial conditions
q(1,1) = offset(1);
dq(1,1) = 0*A*w;

q(2,1) = offset(2);
dq(2,1) = 0*A*w;

% Environment dynamics
K_env = 0;
B_env = 20;  %N.s/m
y_w = 0.5;

% Impedance controller parameters
K_d = 50;	%N/m
B_d = 0;	%N.s/m
M_d = 0;	%N.s^2/m

% PID Gains
k_p = diag([20 20]);	%N/m
k_d = diag([0 0]);      %N.s/m
k_i = diag([0 0]);      %N/m.s

j = 1;
g = 0;

% Simulation (Forward Dynamics)
for i = 1 : length(t) - 1
    % Matrices
    [H, G, C, F] = TwoDOF_InvDyn(q(:,i), dq(:,i), l, m, g);
    
    % Gravity compensator
    T_g(:, i) = G;
    
    % Jacobian (T = J'*F)
    Jacobian = JacobianMatrix(l,q(:,i),dof);
    
    % End-effector coordinates
    [coord_x, coord_y] = L_forward(l, q(:,i), dof);
    X(:,i) = [coord_x; coord_y];
    x(:,i) = coord_x;
    y(:,i) = coord_y;
    
    % End-effector velocities
    dX(:,i) = Jacobian*dq(:,i);
    
    if rem(i,dt_c/dt) == 0
        
        t_c(j) = t(i);
        dt = dt_c;
        
        T_r(1,i) = K_d*(q_d(1,i) - q(1,i)) - B_d*(dq(1,i)) - M_d*ddq(1,i);
        T_r(2,i) = K_d*(q_d(2,i) - q(2,i)) - B_d*(dq(2,i)) - M_d*ddq(1,2);
        T_r(:,i) = [T_r(1,i); T_r(2,i)];
        
        T_ext(1,i) = 0;
        T_ext(2,i) = K_env*(q(2,i)) + B_env*dq(2,i);

        % PID Force control
        
        %T_p(:, i) = H*ddq_d(:,i) + C*dq_m(:, i) + G + F*dq_m(:, i) + T_ext(:,i);
        %T_d(:,i) = T_p(:,i);
        
        T_error(:,i) = T_r(:,i) + T_d(:,i) - T_ext(:,i);
        
        if i ~= 1
            int_T_error(:, i) = int_T_error(:, i - 1) + (T_error(:,i) + T_error(:, i - 1))*dt/2;
            %dT_error(:,i) = (T_error(:,i) - T_error(:,i - 1))/dt;
        end
        
        T_a(:,i) = k_p * T_error(:,i)  + k_d * dT_error(:,i) + k_i * int_T_error(:, i) + T_g(:, i);
        T_a(:, i) = Saturation( T_a(:, i), torque_max, torque_min);
        
    else
        % Zero-Order Holder (ZOH)
        if i > 1
            T_r(:,i) = T_r(:, i - 1);
            T_ext(:, i) = T_ext(:, i - 1);
            T_error(:, i) = T_error(:, i - 1);
            T_a(: , i) = T_a(:, i - 1);
        end
    end
    dt = t(2) - t(1);
    
    % Simulation
    ddq(:, i) = H\(T_d(:, i) + T_a(:,i) - C*dq(:, i) - G - F*dq(:, i)  - isBackdrivable*T_ext(:,i));
    dq(:, i + 1) = dq(:, i) + ddq(:, i)*dt;
    q(:, i + 1) = q(:, i) + dq(:, i)*dt + 0.5*ddq(:, i)*dt*dt;
    
end

% Animation
if(play)
    run('Animation_TwoDOF_Underwater.m')
end

% Plots
run('TwoDOF_Plots.m')