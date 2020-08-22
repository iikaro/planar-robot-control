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
SEA = 0;

% Model parameters
[t, dt] = SimulationTime(0,6,5e-3);

%% Variables
run('Adm_TwoDOF_Variables.m')

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

% Desired trajectory
for i = 1 : length(t)
    if t(i) >= 1, q_d(1,i) = 1 + offset(1); end
    
    if t(i) >= 4, q_d(1,i)  = A*cos(w*t(i)) + offset(1); end
    
    if t(i) >= 5, q_d(1,i) = 0 + offset(1); end
end

x_d = l(1)*cos(q_d(1,:)) + l(2)*cos(q_d(1,:) + q_d(2,:));
y_d = l(1)*sin(q_d(1,:)) + l(2)*sin(q_d(1,:) + q_d(2,:));

% Initial conditions
q_m(1,1) = offset(1);
dq_m(1,1) = A*w;

q_m(2,1) = offset(2);
dq_m(2,1) = 0*A*w;

%% Force sensor/Environment dynamics
x_w = 0.5;	%m
dx_w = 0;   %m/s
K_fs = 100;	%N/m        %(Ott, 2010)
B_fs = 10;  %N.s/m      

%Wall is modeled as spring-damper system in Clover, 1999
% In Seraji 1994, B_env = 10, K_env = 100. K_env varia depois para 25.

%% Admittance controller parameters (stiff wall)
K_d = 15;	%N/m
B_d = 5;	%N.s/m
M_d = 0.01;	%N.s^2/m

%% PID Gains (300, 15, 0, sem SEA)
k_p = diag([1500 100]);	%N/m
k_d = diag([30 10]);	%N.s/m
k_i = diag([10 0]);      %N/m.s

%% Simulation (Forward Dynamics)
for i = 1 : length(t) - 1
    % Matrices
    [H, G, C, F] = TwoDOF_InvDyn(q_m(:,i), dq_m(:,i), l, m, g);
    
    % Gravity compensator
    T_g(:, i) = G;
    
    % Jacobian (T = J'*F)
    Jacobian = JacobianMatrix(l,q_m(:,i),dof);
    
    % End-effector coordinates
    [coord_x, coord_y] = L_forward(l, q_m(:,i), dof);
    x(:,i) = [coord_x; coord_y];
    x_m(:,i) = coord_x;
    y_m(:,i) = coord_y;
    % End-effector velocities
    dx(:,i) = Jacobian*dq_m(:,i);
    
    %% Wall detection
    if(OnCollisionEnter(x(:,i), x_w))
        F_int = OnCollisionStay(x(:,i), dx(:,i), x_w, dx_w, K_fs, B_fs, 1, SEA, q_m(:,i), dq_m(:,i), q_d(:,i), l);
        F_int = [F_int(2); F_int(1)];
        F_ext(:,i) = -F_int;
    end

    F_error(:,i) = F_d(:,i) - F_ext(:,i);
    
    %% Admittance Control Law
    if i == 1 && K_d ~= 0
        x_v(:,i) = F_error(:,i)./K_d;
    end
    
    if i > 1 && B_d ~= 0
        x_v(:,i) = (F_error(:,i) + x_v(:,i - 1).*B_d/dt)./(B_d/dt + K_d);
    end
    
    if i > 2
        x_v(:,i) = (F_error(:,i) + x_v(:,i - 1).*(2*M_d/(dt^2) + B_d/dt) - x_v(:,i - 2).*M_d/(dt^2))./(M_d/(dt^2) + B_d/dt + K_d);
    end
    
    %q_v(2,i) = atan2(x_v(2,i), x_w);
    
    %q_v(2,i) = acos(x_v(2,i)/l(2)) - q_m(1,i);
    
    if(F_error(1,i) ~= 0 || F_error(2,i) ~= 0)
        q_v(2,i) = atan2(x_v(2,i),x_v(1,i)) + q_m(1,i);
        q_v_pure(i) = atan2(x_v(2,i),x_v(1,i));
    end
    
    if i > 1
        dq_v(2,i) = ( q_v(2,i) - q_v(2,i-1) )/ dt;
    end
    
    
    %% PID Position control
    q_r(:,i) = q_d(:,i) + q_v(:,i);
    q_error(:, i) = q_r(:,i) - q_m(:,i) ;
    q_error_pure(:,i) = q_d(:,i) - q_m(:,i);
    dq_error(:, i) = dq_d(:,i) - dq_m(:,i) + 0*dq_v(:,i);

    if i ~= 1
        int_error(:, i) = int_error(:, i - 1) + (q_error(:,i) + q_error(:, i - 1))*dt/2;
    end
    
    T_a(:, i) = k_p * q_error(:, i) + k_d * dq_error(:, i) + k_i * int_error(:, i) + T_g(:, i);
    
    %% Simulation
    ddq_m(:, i) = H\(T_d(:, i) + T_a(:,i) - C*dq_m(:, i) - G - F*dq_m(:, i)  - Jacobian'*F_ext(:,i));
    dq_m(:, i + 1) = dq_m(:, i) + ddq_m(:, i)*dt;
    q_m(:, i + 1) = q_m(:, i) + dq_m(:, i)*dt + 0.5*ddq_m(:, i)*dt*dt;
    
end

x_m(end) = x_m(end-1);
y_m(end) = y_m(end-1);

%% Animation
q = q_m;
L = l;
if(play)
    run('Animation_Adm_TwoDOF.m')
end

%% Plots
run('Adm_TwoDOF_Plots.m')

return
%% Analysis
run('Analysis.m')

%% Filtering data
windowSize = 2; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
x = q_v;
LPB = LowPassButterworth
%%
y = filter(LPB,x);

figure
plot(t,x)
hold on
plot(t,y)
legend('Input Data','Filtered Data')
%%
figure
Fs = 1/dt;
Hpsd = dspdata.psd(x(1,:),'Fs',Fs)
plot(Hpsd)