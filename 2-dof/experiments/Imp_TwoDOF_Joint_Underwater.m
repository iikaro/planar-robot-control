%% Control - 2 DoF Robotic Arm
clc;
clear all;
close all;
addpath('..\..\shared-functions')

%% Play animation?
play = 0;
SEA = 0;

%% Simulation parameters
dt = 0.005;
tf = 10;
ti = 0;
t = linspace(0, 10, tf*1/dt+1);

%% Variables
run('Adm_TwoDOF_Variables.m')
dF_error = zeros(2,length(t));
F_r = zeros(2,length(t));
dtau_error = zeros(2,length(t));
%% Desired trajectory
offset = [-pi/2; -pi/8];
A = 1.5;
w = 0.5*pi/2;

% First joint
q_d(1,:) = 0*A*sin(w*t) + offset(1);
dq_d(1,:) = 0*w*A*cos(w*t);
% Second joint
q_d(2,:) = A*sin(w*t) + offset(2);
dq_d(2,:) = 0*w*A*cos(w*t);
ddq_d(2,:) = -w*w*A*sin(w*t);

% Desired trajectory
for i = 1 : length(t)
    if t(i) >= 2, q_d(2,i) = 1 - offset(2); end

    %if t(i) >= 4, q_d(2,i)  = A*cos(w*t(i)) + offset(1); end

    %if t(i) >= 5, q_d(2,i) = 0 + offset(1); end
end

x_d = l(1)*cos(q_d(1,:)) + l(2)*cos(q_d(1,:) + q_d(2,:));
y_d = l(1)*sin(q_d(1,:)) + l(2)*sin(q_d(1,:) + q_d(2,:));

% Initial conditions
q_m(1,1) = offset(1);
dq_m(1,1) = 0*A*w;

q_m(2,1) = offset(2);
dq_m(2,1) = 0*A*w;

% figure
% plot(t,q_d')
% hold on
% plot(dq_d')
% plot(ddq_d')

%% Force sensor/Environment dynamics
K_fs = 0;  %N/m
B_fs = 3;   %N.s/m

%% Impedance controller parameters (stiff wall)
K_d = 10;	%N/m
B_d = 0;	%N.s/m
M_d = 0;	%N.s^2/m

%% PID Gains (300, 15, 0, sem SEA)
k_p = diag([30 10]);	%N/m
k_d = diag([1 1]);      %N.s/m
k_i = diag([0 0]);      %N/m.s

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
    
    %F_r_x(i) = K_d*(x_d(i) - x_m(i)) - B_d*(dx(1,i));
    %F_r_y(i) = K_d*(y_d(i) - y_m(i)) - B_d*(dx(2,i));
    
    %F_r(:,i) = [F_r_x(i); F_r_y(i)];
    
    tau_r(1,i) = K_d*(q_d(1,i) - q_m(1,i)) - B_d*(dq_m(1,i));
    tau_r(2,i) = K_d*(q_d(2,i) - q_m(2,i)) - B_d*(dq_m(2,i));
    tau_r(:,i) = [tau_r(1,i); tau_r(2,i)];
    
    %% Wall detection
    %{
    if(OnCollisionEnter(x(:,i), x_w))
        F_int = OnCollisionStay(x(:,i), dx(:,i), x_w, dx_w, K_fs, B_fs, 1, SEA, q_m(:,i), dq_m(:,i), q_d(:,i), l);
        F_int = [F_int(2); F_int(1)];
        F_ext(:,i) = F_int;
    end
    
    if(x_m(i) > x_w)
        F_ext(:,i) = [K_fs*(x_m(i) - x_w) + B_fs*dx(2,i); 0];
        if F_ext(1,i) < 0
            F_ext(1,i) = 0;
        end
    end
    %}
    tau_ext(1,i) = 0;
    
    tau_ext(2,i) = K_fs*(q_m(2,i)) + B_fs*dq_m(2,i);
    
    
    %% PID Force control
    %F_error(:,i) = F_r(:,i) + F_d(:,i) - F_ext(:,i);
    
    tau_p(:, i) = H*ddq_d(:,i) + C*dq_m(:, i) + G + F*dq_m(:, i) + tau_ext(:,i);
    T_d(:,i) = tau_p(:,i);
    
    tau_error(:,i) = tau_r(:,i) + [0;0] - tau_ext(:,i);
    
    if i ~= 1
        %int_error(:, i) = int_error(:, i - 1) + (F_error(:,i) + F_error(:, i - 1))*dt/2;
        dF_error(:,i) = ( F_error(:,i) - F_error(:,i-1) ) / dt;
        dtau_error(:,i) = (tau_error(:,i) - tau_error(:,i - 1))/dt;
    end
    
    %T_a(:, i) = k_p * q_error(:, i) + k_d * dq_error(:, i) + k_i * int_error(:, i) + T_g(:, i);
    
    %T_a(:,i) = Jacobian' * k_p * F_error(:,i)  + Jacobian' * k_d * dF_error(:,i) + T_g(:, i);
    
    T_a(:,i) = k_p * tau_error(:,i)  + k_d * dtau_error(:,i) + T_g(:, i);
    
    %T_a(:,i) = Jacobian'*F_r(:,i);
    %% Simulation
    ddq_m(:, i) = H\(T_d(:, i) + T_a(:,i) - C*dq_m(:, i) - G - F*dq_m(:, i)  - tau_ext(:,i));
    dq_m(:, i + 1) = dq_m(:, i) + ddq_m(:, i)*dt;
    q_m(:, i + 1) = q_m(:, i) + dq_m(:, i)*dt + 0.5*ddq_m(:, i)*dt*dt;
    
end

x_m(end) = x_m(end-1);
y_m(end) = y_m(end-1);

%% Animation
L = l;
q = q_m;
if(play)
    run('Animation_Adm_TwoDOF.m')
end
%%
figure
plot(tau_r')
title('Impedance forces')
figure
plot(tau_ext')
title('Contact forces')
figure
plot(t,x_d,t,x_m)
title('X trajectory (desired and real)')
figure
plot(t,y_d,t,y_m)
title('Y trajectory (desired and real)')
figure
plot(t,[A*sin(w*t) + offset(2);offset(1)+0*t],t,q_m)
title('desired and real angles')
legend('desired 1','desired 2','joint 1','joint 2')
%% Plots
%{
run('Adm_TwoDOF_Plots.m')

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
%}