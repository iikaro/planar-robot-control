% Control - 2 DoF Robotic Leg Underwater
clc;
clear all;
close all;
addpath('..\..\shared-functions')

% Play animation?
play = 0;
isBackdrivable = 0;

% Model parameters
[t, dt] = SimulationTime(0,6,5e-3);

% Variables
run('TwoDOF_Variables.m')

% Desired trajectory
offset = [0; -pi/2];
A = 45*pi/180;
w = 1*pi;

% First joint
q_d(1,:) = 0*A*sin(w*t) + offset(1);
dq_d(1,:) = 0*w*A*cos(w*t);

% Second joint
q_d(2,:) = A*sin(w*t) + offset(2);
dq_d(2,:) = w*A*cos(w*t);

for i = 1 : length(t)
    if t(i) >= 2.5, q_d(2,i) = q_d(2,i-1); end
end

% Cartesian coordinates
x_d = l(1)*cos(q_d(1,:)) + l(2)*cos(q_d(1,:) + q_d(2,:));
y_d = l(1)*sin(q_d(1,:)) + l(2)*sin(q_d(1,:) + q_d(2,:));

% Initial conditions
q(1,1) = offset(1);
dq(1,1) = 0*A*w;

q(2,1) = offset(2);
dq(2,1) = A*w;

% Environment dynamics
y_w = 0.5;  %environment interface
K_env = 0*diag([100, 100]);
B_env = diag([20, 20]);  %N.s/m

% Follow strictly trajectory
K_d = 50;	%N/m
B_d = 0;	%N.s/m
M_d = 0;	%N.s^2/m

% PID Gains (300, 15, 0, without SEA)
k_p = diag([100 200]);	%N/m
k_d = diag([20 0]);	%N.s/m
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
    x(:,i) = coord_x;
    y(:,i) = coord_y;
    
    % End-effector velocities
    dX(:,i) = Jacobian*dq(:,i);
    
    % Drag calculation
    T_ext(:, i) = K_env*(q(:,i)) + B_env*dq(:,i); %always positive
    T_ext(:, i) = [0; T_ext(2,i)];
    %tau_ext(:, i) = [0; -20*w*w*1*cos(w*t(i))];
    T_error(:, i) = T_d(:, i) - T_ext(:, i);
    
    % Admittance Control Law
    if i == 1 && K_d ~= 0
        q_v(:,i) = T_error(:,i)./K_d; end
    
    if i > 1 && B_d ~= 0
        q_v(:,i) = (T_error(:,i) + q_v(:,i - 1).*B_d/dt)./(B_d/dt + K_d); end
    
    if i > 2
        q_v(:,i) = (T_error(:,i) + q_v(:,i - 1).*(2*M_d/(dt^2) + B_d/dt) - q_v(:,i - 2).*M_d/(dt^2))./(M_d/(dt^2) + B_d/dt + K_d); end
    
    if i > 1
        dq_v(2,i) = ( q_v(2,i) - q_v(2,i-1) )/ dt;
    end
    
    % PID Position control
    q_r(:,i) = q_d(:,i) + q_v(:,i);
    q_error(:, i) = q_r(:,i) - q(:,i) ;
    dq_error(:, i) = dq_d(:,i) - dq(:,i);
    
    if i ~= 1
        int_error(:, i) = int_error(:, i - 1) + (q_error(:,i) + q_error(:, i - 1))*dt/2;
    end
    
    T_a(:, i) = k_p * q_error(:, i) + k_d * dq_error(:, i) + k_i * int_error(:, i) + T_g(:, i);
    
    T_a(:, 2) = Saturation( T_a(:, 2), torque_max, torque_min);
    
    % Simulation
    ddq(:, i) = H\(T_d(:, i) + T_a(:,i) - C*dq(:, i) - G - F*dq(:, i)  - isBackdrivable*T_ext(:,i));
    dq(:, i + 1) = dq(:, i) + ddq(:, i)*dt;
    q(:, i + 1) = q(:, i) + dq(:, i)*dt + 0.5*ddq(:, i)*dt*dt;
    
end

% Animation
if(play)
    run('Animation_TwoDOF_Underwater.m')
end

%
figure
plot(t,q_d',t, q')
%legend('theta 1 virtual','theta 2 virtual','theta 1 error','theta 2 error')

% Plots
run('TwoDOF_Plots.m')