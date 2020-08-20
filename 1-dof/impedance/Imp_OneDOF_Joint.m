% Control - 1 DoF Robotic Arm
%{
Departamento de Engenharia Mec�nica
Escola de Engenharia de S�o Carlos - Universidade de S�o Paulo
�rea de Concentra��o: Din�mica e Mecatr�nica
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

% Desired trajectory
q_d = 1*sin(pi*t)';

% Environment
K_env = 5e3;
B_env = 0;

% Obstacle
angle = 30*pi/180;
wall = sin(angle)*L;

% Impedance Control
K_imp = 20;
B_imp = 1;

% PID Force Control Gains
Kp = 100;
Kd = 0.5;
Ki = 0;

% Simulation
for i = 1 : length(t) - 1
    
    % Desired Torque (Impedance Control)
    tau_r(i) = K_imp*(q_d(i) - q(i)) - B_imp*(dq(i));
    
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
    
    % Control signal (=~ torque)
    tau_a(i) = Kp * tau_error(i) + Kd * dtau_error(i) + Ki * tau_int_error(i) + (L/2)*m*g*cos(q(i));
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