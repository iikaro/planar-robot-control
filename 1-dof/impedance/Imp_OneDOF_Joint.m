% Control - 1 DoF Robotic Arm
%{
Departamento de Engenharia Mecânica
Escola de Engenharia de São Carlos - Universidade de São Paulo
Área de Concentração: Dinâmica e Mecatrônica
%}
clc;
clear all;
addpath('..\..\shared-functions')

% Play animation?
play = 1;

% Model parameters
[t, dt] = SimulationTime(0,6,5e-3);
run('Variables_1DoF.m')

% Desired trajectory
q_d = 1*sin(pi*t)';

% Environment
K_env = 5e3;
B_env = 10;

% Obstacle
angle = 28*pi/180;
wall = sin(angle)*L;

% Impedance Control
K_d = 200;
B_d = 1;
M_d = 0;

% PID Force Control Gains
k_p = 5;
k_d = 0.5;
k_i = 0;

% Simulation
for i = 2 : length(t) - 1
    
    % Desired Torque (Impedance Control)
    tau_r(i) = K_d*(q_d(i) - q(i)) - B_d*(dq(i)) - M_d*ddq(i-1);
    
    % Environment Detection
    if(q(i) > angle)
        tau_ext(i) = K_env*(q(i) - angle) + B_env*dq(i);
    end
    
    % Torque Error
    tau_error(i) = tau_d(i) + tau_r(i) - tau_ext(i);
    
    if i ~= 1
        dtau_error(i) = (tau_error(i) - tau_error(i - 1))/dt;
        tau_int_error(i) = tau_int_error(i - 1) + (tau_error(i) + tau_error(i - 1))*dt/2;
    end
    
    % Control signal
    tau_a(i) = k_p * tau_error(i) + k_d * dtau_error(i) + k_i * tau_int_error(i) + (L/2)*m*g*cos(q(i));
    tau_a(i) = Saturation(tau_a(i), torque_max, torque_min);
    
    % Outputs
    ddq(i) = (1/J)*(tau_a(i) - (L/2)*m*g*cos(q(i)) - B*dq(i) - K*q(i));
    dq(i + 1) = dq(i) + ddq(i)*dt;
    q(i + 1) = q(i) + dq(i)*dt + 0.5*ddq(i)*dt*dt;
end

% Graphic Simulation
if(play)
    Animation(q, q_d, L, t, offset, wall, dof);
end