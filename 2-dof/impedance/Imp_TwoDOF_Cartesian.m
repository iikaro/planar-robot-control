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
isImpedance = 1;
isTaskSpace = 1;

% Play animation?
play = 1;
isBackdrivable = 1;

% Model parameters
[t, dt] = SimulationTime(0,6,5e-3);
% Variables
run('TwoDOF_Variables.m')

% Desired trajectory
offset = [-pi/3; -pi/8];
A = 60*pi/180;
w = 0.5*pi;

% First joint
q_d(1,:) = A*sin(w*t) + offset(1);
dq_d(1,:) = 0*w*A*cos(w*t);
% Second joint
q_d(2,:) = 0*A*sin(w*t) + offset(2);
dq_d(2,:) = 0*w*A*cos(w*t);

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
dq(1,1) = A*w;

q(2,1) = offset(2);
dq(2,1) = 0*A*w;

% Environment dynamics
x_w = 0.6;	%m
dx_w = 0;   %m/s
K_env = 1e4;  %N/m
B_env = 10;   %N.s/m

% Impedance controller parameters (stiff wall)
K_d = 10*diag(2);	%N/m
B_d = 5*diag(2);	%N.s/m
M_d = 0*diag(2);	%N.s^2/m

% PID Gains (300, 15, 0, without SEA)
k_p = diag([20 20]);	%N/m
k_d = diag([0 0]);      %N.s/m
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
    
    F_r (:,i) = K_d * (X_d(:,i) - X(:,i) ) - B_d*dX(:,i);
    
    % Wall detection
    if(x(i) > x_w)
        F_ext(:,i) = [K_env*(x(i) - x_w) + B_env*dX(2,i); 0];
        F_ext(2,i) = 0;
        
        if F_ext(1,i) < 0
            F_ext(1,i) = 0;
        end

        if i > 1
            %F_ext(:,i) = K_env*(2*(x_m(i) - x_w) - 1.5*(x_m(i-1) - x_w));
            F_ext_filtered(:,i) = 2*F_ext(:,i) - 1.5*F_ext(:,i - 1);
            
            if F_ext_filtered(1,i) < 0 || F_ext_filtered(2,i) < 0
                %F_ext_filtered(:,i) = [0;0];
            end
        end
        if i > 2
            %F_ext(:,i) = K_env*(2*(x_m(i) - x_w) - 1.5*(x_m(i-1) - x_w) + 0.5*(x_m(i-2) - x_w));
            F_ext_filtered(:,i) = 2*F_ext(:,i) - 1.5*F_ext(:,i - 1) + 0.5*F_ext(:,i - 2);
            
            if F_ext_filtered(1,i) < 0 || F_ext_filtered(2,i) < 0
                %F_ext_filtered(:,i) = [0;0];
                
            end
            
        end
    end
    
    
    % PID Force control
    %F_error(:,i) = F_r(:,i) + F_d(:,i) - F_ext(:,i);
    F_error(:,i) = F_r(:,i) + F_d(:,i) - F_ext_filtered(:,i);
    
    if i ~= 1
        %int_error(:, i) = int_error(:, i - 1) + (F_error(:,i) + F_error(:, i - 1))*dt/2;
        dF_error(:,i) = ( F_error(:,i) - F_error(:,i-1) ) / dt;
    end
    
    %T_a(:, i) = k_p * q_error(:, i) + k_d * dq_error(:, i) + k_i * int_error(:, i) + T_g(:, i);
    
    T_a(:,i) = Jacobian' * k_p * F_error(:,i)  + Jacobian' * k_d * dF_error(:,i) + T_g(:, i);
    T_a(:, i) = Saturation( T_a(:, i), torque_max, torque_min);
    
    % Simulation
    ddq(:, i) = H\(T_d(:, i) + T_a(:,i) - C*dq(:, i) - G - F*dq(:, i) - isBackdrivable*Jacobian'*F_ext_filtered(:,i));
    dq(:, i + 1) = dq(:, i) + ddq(:, i)*dt;
    q(:, i + 1) = q(:, i) + dq(:, i)*dt + 0.5*ddq(:, i)*dt*dt;
    
end

% Animation
if(play)
    run('Animation_2DoF.m')
end
%
run('TwoDOF_Plots.m')
return
figure
plot(t,F_r')
title('Impedance forces')
figure
plot(t,T_a')
title('Actuator forces')
figure
plot(t,F_ext')
title('Contact forces')
figure
plot(t,x_d,t,x)
title('X trajectory (desired and real)')
figure
plot(t,y_d,t,y)
title('Y trajectory (desired and real)')