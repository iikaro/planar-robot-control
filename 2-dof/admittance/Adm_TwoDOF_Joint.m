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
isTaskSpace = 0;
isImpedance = 0;
isBackdrivable = 0;

% Play animation?
play = 1;

% Model parameters
[t, dt] = SimulationTime(0,6,5e-3);

% Variables
run('TwoDOF_Variables.m')

% Desired trajectory
offset = [-pi/2; -pi/8];
A = 1.5;
w = 0.5*pi/2;

% First joint
q_d(1,:) = A*sin(w*t) + offset(1);
dq_d(1,:) = 0*w*A*cos(w*t);
% Second joint
q_d(2,:) = 0*A*sin(w*t) + offset(2);
dq_d(2,:) = 0*w*A*cos(w*t);

% Desired trajectory
%{
for i = 1 : length(t)
    if t(i) >= 1, q_d(1,i) = 1 + offset(1); end
    
    if t(i) >= 4, q_d(1,i)  = A*cos(w*t(i)) + offset(1); end
    
    if t(i) >= 5, q_d(1,i) = 0 + offset(1); end
end
%}
x_d = l(1)*cos(q_d(1,:)) + l(2)*cos(q_d(1,:) + q_d(2,:));
y_d = l(1)*sin(q_d(1,:)) + l(2)*sin(q_d(1,:) + q_d(2,:));
X_d = [x_d; y_d];

% Initial conditions
q(1,1) = offset(1);
dq(1,1) = 0*A*w;

q(2,1) = offset(2);
dq(2,1) = A*w;

% Environment dynamics
x_w = 0.6;
dx_w = 0;   %m/s
K_env = diag([1e2, 0]);	%N/m        %(Ott, 2010)
B_env = diag([5, 0]);  %N.s/m      

% Wall is modeled as spring-damper system in Clover, 1999
% In Seraji 1994, B_env = 10, K_env = 100. K_env varia depois para 25.

% Admittance controller parameters (stiff wall)
K_d = 5;	%N/m
B_d = 1;	%N.s/m
M_d = 0.01;	%N.s^2/m

% PID Gains (300, 15, 0, without SEA)
k_p = diag([200 150]);	%N/m
k_d = diag([20 15]);	%N.s/m
k_i = diag([0 0]);      %N/m.s

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
    
     % Wall detection
    if x(1,i) > x_w(1)
        F_ext(:,i) = (K_env*(X(:,i) - x_w) + B_env*dX(:,i));
        
        if F_ext(1,i) < 0 || F_ext(2,i) < 0
            F_ext(:,i) = [0;0];
        end
    end
    T_ext(:,i) = Jacobian'*F_ext(:,i);
    
    %T_r(:,i) = K_sea*(q_d(:,i) - q(:,i)) - B_sea*(dq(:,i));
    
    T_error(:,i) = T_d(:,i) - T_ext(:,i);
    
    % Admittance Control Law
    if i == 1 && K_d ~= 0
        q_v(:,i) = T_error(:,i)./K_d;
    end
    
    if i > 1 && B_d ~= 0
        q_v(:,i) = (T_error(:,i) + q_v(:,i - 1).*B_d/dt)./(B_d/dt + K_d);
    end
    
    if i > 2
        q_v(:,i) = (T_error(:,i) + q_v(:,i - 1).*(2*M_d/(dt^2) + B_d/dt) - q_v(:,i - 2).*M_d/(dt^2))./(M_d/(dt^2) + B_d/dt + K_d);
    end
    
    q_v(:,i) = [0; q_v(1,i) + q_v(2,i)];
    % PID Position control
    q_r(:,i) = q_d(:,i) + q_v(:,i);
    q_error(:, i) = q_r(:,i) - q(:,i) ;
    dq_error(:, i) = dq_d(:,i) - dq(:,i) + 0*dq_v(:,i);

    if i ~= 1
        int_error(:, i) = int_error(:, i - 1) + (q_error(:,i) + q_error(:, i - 1))*dt/2;
    end
    
    T_a(:, i) = k_p * q_error(:, i) + k_d * dq_error(:, i) + k_i * int_error(:, i) + T_g(:, i);
    T_a(:, i) = Saturation( T_a(:, i), torque_max, torque_min);
    
    % Simulation
    ddq(:, i) = H\(T_d(:, i) + T_a(:,i) - C*dq(:, i) - G - F*dq(:, i)  - isBackdrivable*T_ext(:,i));
    dq(:, i + 1) = dq(:, i) + ddq(:, i)*dt;
    q(:, i + 1) = q(:, i) + dq(:, i)*dt + 0.5*ddq(:, i)*dt*dt;
    
end

% Animation
if(play)
    run('Animation_2DoF.m')
end
% Plots
run('TwoDOF_Plots.m')