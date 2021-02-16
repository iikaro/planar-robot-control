%% Control - 2 DoF Robotic Arm
%{
Departamento de Engenharia Mecânica
Escola de Engenharia de São Carlos - Universidade de São Paulo
Área de Concentração: Dinâmica e Mecatrônica
%}
clc;
clear all;
close all;
addpath('..\..\shared-functions')

%% Play animation?
play = 1;
dof = 2;
% Model parameters
dt = 1e-3;
t = linspace(0,5,1/dt+1);

%% Variables
q_d = zeros(2,length(t));
dq_d = zeros(2,length(t));
ddq_d = zeros(2,length(t));
% State
q = zeros(2,length(t));
dq = zeros(2,length(t));
ddq = zeros(2,length(t));
% State error
int_error = zeros(2,length(t));
q_error = zeros(2,length(t));
dq_error = zeros(2,length(t));
% Torques
T_d = zeros(2,length(t));       %direct dynamics torque
T_a = zeros(2,length(t));       %actuator torque
T_g = zeros(2,length(t));       %gravity compensator

% Model parameters
m = [10, 5];
l = [0.5, 0.5];

gear_ratio = 150;       %dimensionless
nominal_current = 3.17; %A
nominal_torque = 187e-3;%N.m
torque_cte = 60.3e-3;   %N.m/A
eff = 0.92;             %dimensionless

torque_max = nominal_current*torque_cte*eff*gear_ratio*ones(2,1); %N.m
torque_min = -torque_max;	%N.m

g = -9.81;

%% Desired trajectory
offset = [-pi/2; -pi/8];
A = 1;
w = 0.5*pi;

% First joint
q_d(1,:) = A*sin(w*t) + offset(1);
dq_d(1,:) = w*A*cos(w*t);
% Second joint
q_d(2,:) = 0*A*sin(w*t) + offset(2);
dq_d(2,:) = 0*w*A*cos(w*t);

% Initial conditions
q(1,1) = offset(1);
dq(1,1) = A*w;

q(2,1) = offset(2);
dq(2,1) = 0*A*w;

%% PID
k_p = diag([1500 100]);
k_d = diag([30 10]);
k_i = diag([10 0]);

%% Simulation (Forward Dynamics)
for i = 1 : length(t) - 1
    
    % Matrices
    I = [1;1];
    
    H11 = m(1)*l(1)/2^2 + m(2)*(l(1)^2 + l(2)/2^2 + 2*l(1)*l(2)/2*cos(q(2,i))) + I(1) + I(2);
    H12 = m(2)*(l(2)/2^2 + l(1)*l(2)/2*cos(q(2,i))) + I(2);
    H21 = m(2)*(l(2)/2^2 + l(1)*l(2)/2*cos(q(2,i))) + I(2);
    H22 = m(2)*l(2)/2^2 + I(2);
    
    H = [H11 H12; H21 H22];
    
    h = sin(q(2,i))*m(2)*l(1)*l(2)/2;
    
    C11 = -h*dq(2,i);
    C12 = -h*(dq(1,i) + dq(2,i));
    C21 = h*dq(1,i);
    C22 = 0;
    
    C = [C11 C12; C21 C22];
    
    G11 = (m(1)*g*l(1)/2 + m(2)*g*l(1))*cos(q(1,i)) + m(2)*g*l(2)/2*cos(q(1,i) + q(2,i));
    G21 = m(2)*g*l(2)/2*cos(q(1,i) + q(2,i));
    
    G = [G11;
        G21];
    
    F11 = 0.1;
    F21 = 0.1;
    
    F = [F11, F21];
    
    % Gravity compensator
    T_g(:, i) = G;
    
    %% PID Position control
    q_error(:, i) = q_d(:,i) - q(:,i);
    dq_error(:, i) = dq_d(:,i) - dq(:,i);
    
    if i ~= 1
        int_error(:, i) = int_error(:, i - 1) + (q_error(:,i) + q_error(:, i - 1))*dt/2;
    end
    
    T_a(:, i) = k_p * q_error(:, i) + k_d * dq_error(:, i) + k_i * int_error(:, i) + T_g(:, i);
    
    %% Simulation
    ddq(:, i) = inv(H)*(T_a(:,i) - C*dq(:, i) - G - F*dq(:, i));
    dq(:, i + 1) = dq(:, i) + ddq(:, i)*dt;
    q(:, i + 1) = q(:, i) + dq(:, i)*dt + 0.5*ddq(:, i)*dt*dt;
    
end

%% Animation
O = [0 0];  %origin
L = l;

h = figure;

A = L(1)*[cos(q(1,:)); sin(q(1,:))];
B = A + L(2)*[cos(q(2,:) + q(1,:)); sin(q(2,:) + q(1,:))];
D = A + L(2)*[cos(q_d(2,:) + q_d(1,:)); sin(q_d(2,:) + q_d(1,:))];

for i = 1 : length(t)
    if(ishandle(h))
        plot([O(1,1) A(1,i) B(1,i)], [O(1,2) A(2,i) B(2,i)], 'k', 'LineWidth', 3)
        axis([-1.5 1.5 -1.5 1.5])
        hold on
        plot([A(1,i) D(1,i)], [A(2,i) D(2,i)], ':b', 'LineWidth', 1)
        grid on
        hold off
        pause(5e-3)
    end
end