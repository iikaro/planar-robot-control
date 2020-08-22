% Admittance Control - 1 DoF Robotic Arm
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

% Model parameters
[t, dt] = SimulationTime(0,6,5e-3);
run('Variables_1DoF.m')

% Desired Trajectory
q_d = 1*sin(pi*t)';

% PID Position Control 
k_p = 500;
k_d = 50;
k_i = 0;

% Environment
K_env = 5;
B_env = 1;

% Obstacle
angle = 0.5;
wall = L*sin(angle);

% Admittance Control Parameters
K_d = 1e3;             %N/rad
B_d = 10;             %N.s/rad
M_d = 0;             %%N.s^2/rad

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
        
    % Wall detection
    if(y > wall)
        F_ext(i) = K_env*(y - wall) + B_env*dy(i);
       
        if(F_ext(i) < 0)
            F_ext(i) = 0;
        end
 
        F_error(:,i) = [0; -F_ext(i)];
    end
    
    % Admittance Control Law
    if i == 1
        x_r(:,i) = F_error(:,i)./K_d;
    end
    
    if i > 1
        x_r(:,i) = (F_error(:,i) + x_r(:,i - 1).*B_d/dt)./(B_d/dt + K_d);
    end
    
    if i > 2
        x_r(:,i) = (F_error(:,i) + x_r(:,i - 1).*(2*M_d/(dt^2) + B_d/dt) + x_r(:,i - 2).*M_d/(dt^2))./(M_d/(dt^2) + B_d/dt + K_d);
    end
    
    % Inverse Kinematics
    if (y > wall)
        q_r(i) = asin((x_r(2,i) - y)/L); 
    end
    
    % Inner position control loop
    q_error(i) = q_d(i) - q(i) + q_r(i);
    
    if i ~= 1
        int_error(i) = int_error(i - 1) + (q_error(i) + q_error(i - 1))*dt/2;
        dq_error(i) = ( q_error(i) - q_error(i-1) ) / dt;
    end
    
    % Control signal (=~ torque)
    tau_a(i) = k_p * q_error(i) + k_d * dq_error(i) + k_i * int_error(i) + (L/2)*m*g*cos(q(i));
    tau_a(i) = Saturation(tau_a(i), torque_max, torque_min);
    
    % Outputs
    ddq(i) = (1/J)*(tau_a(i) + tau_ext(i) - (L/2)*m*g*cos(q(i)) - B*dq(i) - K*q(i));
    dq(i + 1) = dq(i) + ddq(i)*dt;
    q(i + 1) = q(i) + dq(i)*dt + 0.5*ddq(i)*dt*dt;
    
end

% Graphic Simulation
if(play)
    Animation(q, q_d, L, t, offset, wall, dof);
end
