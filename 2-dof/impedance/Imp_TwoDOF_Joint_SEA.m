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
isImpedance = 1;
isBackdrivable = 0;

% Play animation?
play = 0;

% Model parameters
[t, dt] = SimulationTime(0,10,1e-4);
dt_c = 5e-3;

% Variables
run('TwoDOF_Variables.m')
T_p = zeros(2,length(t));
dT_error_filtered = zeros(2,length(t));
% Desired trajectory
offset = [-pi/3; -pi/8];
A = 60*pi/180;
w = 0.5*pi;

% First joint
q_d(1,:) = A*sin(w*t) + offset(1);
dq_d(1,:) = w*A*cos(w*t);
%ddq_d(1,:) = -w*w*A*sin(w*t);
% Second joint
q_d(2,:) = 0*A*sin(w*t) + offset(2);
% dq_d(2,:) = 0*w*A*cos(w*t);
% ddq_d(2,:) = 0*-w*w*A*sin(w*t);

% Desired trajectory
% for i = 1 : length(t)
%     if t(i) >= 1, q_d(1,i) = 1 + offset(1); end
%
%     if t(i) >= 4, q_d(1,i)  = A*cos(w*t(i)) + offset(1); end
%
%     if t(i) >= 5, q_d(1,i) = 0 + offset(1); end
% end

x_d = l(1)*cos(q_d(1,:)) + l(2)*cos(q_d(1,:) + q_d(2,:));
y_d = l(1)*sin(q_d(1,:)) + l(2)*sin(q_d(1,:) + q_d(2,:));
X_d = [x_d; y_d];

% Initial conditions
q(1,1) = offset(1);
dq(1,1) = 0*A*w;

q(2,1) = offset(2);
dq(2,1) = 0*A*w;

%% Force sensor/Environment dynamics
x_w = [0.6; 10];	%m
dx_w = 0;   %m/s
K_env = diag([1e4; 0]);  %N/m
B_env = diag([20; 0]);   %N.s/m

%% Impedance controller parameters (stiff wall)
K_d = 1*diag(2);	%N/m
B_d = 1*diag(2);	%N.s/m
M_d = 0;	%N.s^2/m

%% PID Gains (300, 15, 0, without SEA)
k_p = diag([10 10]);	%N/m
k_d = diag([0 0*0.05]);      %N.s/m
k_i = diag([0 0]);      %N/m.s

g = 0*-9.81;
j = 1;

K_SEA = 104;
B_SEA = 10;

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
    
    
    %T_SEA(:,i) = Saturation( T_a(:, i), torque_max, torque_min);
    
    if rem(i,dt_c/dt) == 0
        t_c(j) = t(i);
        dt = dt_c;
        
        T_r(:,i) = (K_SEA*(q_d(:,i) - q(:,i)) - B_SEA*dq(:,i));
        
        %T_r(:,i) = K_d*(q_d(:,i) - q(:,i)) - B_d*(dq(:,i)) - M_d*ddq(:,i);
        
        % Wall detection
        if x(1,i) > x_w(1)
            F_ext(:,i) = (K_env*(X(:,i) - x_w) + B_env*dX(:,i));
            
            if F_ext(1,i) < 0 || F_ext(2,i) < 0
                F_ext(:,i) = [0;0];
            end
            
            if i > 1
                %F_ext(:,i) = K_env*(2*(x_m(i) - x_w) - 1.5*(x_m(i-1) - x_w));
                F_ext_filtered(:,i) = 2*F_ext(:,i) - 1.5*F_ext(:,i - 1);
                F_ext_filtered(2,i) = 0;
                
                if F_ext_filtered(1,i) < 0
                    F_ext_filtered(:,i) = [0;0];
                end
            end
            
            if i > 2
                %F_ext(:,i) = K_env*(2*(x_m(i) - x_w) - 1.5*(x_m(i-1) - x_w) + 0.5*(x_m(i-2) - x_w));
                F_ext_filtered(:,i) = 2*F_ext(:,i) - 1.5*F_ext(:,i - 1) + 0.5*F_ext(:,i - 2);
                F_ext_filtered(2,i) = 0;
                
                if F_ext_filtered(1,i) < 0
                    F_ext_filtered(:,i) = [0;0];
                    
                end
                
            end
        end
        
        T_ext(:,i) = Jacobian'*F_ext(:,i);
        
        T_ext(:,i) = Jacobian'*F_ext_filtered(:,i);
        
        % PID Force control
        %F_error(:,i) = F_r(:,i) + F_d(:,i) - F_ext(:,i);
        
        
        %T_d(:,i) = tau_p(:,i);
        
        T_error(:,i) = T_r(:,i) + T_d(:,i) - T_ext(:,i);
        
        if i ~= 1
            int_error(:, i) = int_error(:, i - 1) + (T_error(:,i) + T_error(:, i - 1))*dt/2;
            dT_error(:,i) = (T_error(:,i) - T_error(:, i - 1))/dt;
            %dT_error(:,i) = -(T_ext(:,i) - T_ext(:, i - 1))/dt;
        end
        
        dT_error_filtered(:,i) = dT_error(:,i);
        
        %if (i > 1); dT_error_filtered(:,i) = 2*dT_error(:,i) - 1.5*dT_error(:,i - 1); end
        
        %if (i > 2); dT_error_filtered(:,i) = 2*dT_error(:,i) - 1.5*dT_error(:,i - 1) + 0.5*dT_error(:,i - 2);end
        
        %T_a(:, i) = k_p * q_error(:, i) + k_d * dq_error(:, i) + k_i * int_error(:, i) + T_g(:, i);
        %T_a(:,i) = Jacobian' * k_p * F_error(:,i)  + Jacobian' * k_d * dF_error(:,i) + T_g(:, i);
        
        T_a(:,i) = k_p * T_error(:,i) + k_d * dT_error_filtered(:,i) + k_i * int_error(:, i -1);
        T_a(:, i) = Saturation( T_a(:, i), torque_max, torque_min);
        %T_a(:,i) = Jacobian'*F_r(:,i);
        
    else
        % Zero-Order Holder (ZOH)
        if i > 1
            T_r(:,i) = T_r(:, i - 1);
            T_ext(:, i) = T_ext(:, i - 1);
            T_error(:, i) = T_error(:, i - 1);
            dT_error(:, i) = dT_error(:, i - 1);
            dT_error_filtered(:, i) = dT_error_filtered(:, i - 1);
            T_a(: , i) = T_a(:, i - 1);
            T_p(: , i) = T_p(:, i - 1);
            F_ext_filtered(:, i) =  F_ext_filtered(:, i - 1) ;
            F_ext(:, i) =  F_ext(:, i - 1);
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
    run('Animation_2DoF.m')
end

% Plots
run('TwoDOF_Plots.m')