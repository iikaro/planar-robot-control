%% Control - 3 DoF Planar Robotic Arm
%{
Departamento de Engenharia Mecânica
Escola de Engenharia de São Carlos - Universidade de São Paulo
Área de Concentração: Dinâmica e Mecatrônica
Eng. Mecatrônico Ícaro Ostan    nº USP: 8549897
%}
clc;
clear all;
close all;

%% References
% A. A. G. Siqueira et. al., Robust Control of Robots, 2011.

%% Play animation?
play = 1;

%% Enable Control?
control = 1;

%% Simulation Parameters
nt = 1000;  %time steps
t_i = 0;     %initial time
t_f = 50;     %final time
t = linspace(t_i,t_f,nt+1);   %time interval
dt = t(2) - t(1);           %time discretization

offset = ones(3,length(t));

q_d = zeros(3,length(t));
dq_d = zeros(3,length(t));
ddq_d = zeros(3,length(t));

q_m = zeros(3,length(t));
dq_m = zeros(3,length(t));
ddq_m = zeros(3,length(t));

T_d = zeros(3,length(t));
T_g = zeros(3,length(t));
T_dist = zeros(3,length(t));
T_a = zeros(3,length(t));

q_error = zeros(3,length(t));
dq_error = zeros(3,length(t));
int_error = zeros(3,length(t));

%torque_max = 26.378838;
%torque_min = -torque_max;

%% PID Gains
K_p = diag([100 100 50]);
K_d = diag([10 10 10]);
K_i = diag([0 0 0]);

%% Trajectory
offset = -pi/2*offset;
offset = 85*pi/180*offset;
A = 0.5;
w = 0.05*pi;

q_d(1,:) = offset(1,:);
%q_d(2,:) = A*sin(w*t);
%q_d(3,:) = A*sin(w*t);

%dq_d(2,:) = w*A*cos(w*t);
%ddq_d(2,:) = -w*w*A*sin(w*t);

%Initial conditions
%dq_m(3,1) = A*w/2;
%dq_m(2,1) = A*w;
q_m(1,1) = offset(1,1);

%

%% Motor limits & other parameters
%l = [.4 .4 .1];
l = [1, 1, 1];
%F = [0.28, 0.18, 0.10];
F = [0.01, 0.01, 0.01];

%% Disturbance
T_dist(1,:) = 0*(cos(pi/10*t)+0.5*randn(size(t)));
T_dist(2,:) = 0*(cos(pi/10*t)+0.5*randn(size(t)));
T_dist(3,:) = 0*(cos(pi/10*t)+0.5*randn(size(t)));

%% Simulation (Direct Dynamics)
for i = 1 : length(t) - 1
    
    q1 = q_m(1,i);
    q2 = q_m(2,i);
    q3 = q_m(3,i);
    
    dq1 = dq_m(1,i);
    dq2 = dq_m(2,i);
    dq3 = dq_m(3,i);
    
    %% Model parameters
    % ExoTao parameters
    m = [9.9 3.95 1.66];
    l = [.4 .4 .1];
    lc = [.2 .2 .05];
    I = [.2 .053 .0044];
    
    % Icaro's parameters
%     m = [5, 5, 5];
%     l = [1, 1, 1];
%     I = (1/3)*m.*l.^2;
%     lc = l./2;
    
    m1 = m(1);
    m2 = m(2);
    m3 = m(3);
    
    I1 = I(1);
    I2 = I(2);
    I3 = I(3);
    
    l1 = l(1);
    l2 = l(2);
    l3 = l(3);
    
    lc1 = lc(1);
    lc2 = lc(2);
    lc3 = lc(3);
    
    g = -9.81;
    %% Inertia
    M11 = I1 + I2 + I3 + m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2)) + ...
        m3*(l1^2 + l2^2 + lc3^2 + 2*l1*l2*cos(q2) + 2*l1*lc3*cos(q2 + q3) + 2*l2*lc3*cos(q3));
    
    M12 = I2 + I3 + m2*(lc2^2 + 2*l1*lc2*cos(q2)) + ...
        m3*(l2^2 + lc3^2 + l1*l2*cos(q2) + 2*l1*lc3*cos(q2 + q3) + 2*l2*lc3*cos(q3));
    
    M13 = I3 + m3*(lc3^2 + l1*lc3*cos(q2 + q3) + l2*lc3*cos(q3));
    
    M21 = M12;
    
    M22 = I2 + I3 + m2*lc2^2 + m3*(l2^2 + lc3^2 + 2*l2*lc3*cos(q3));
    
    M23 = I3 + m3*(lc3^2 + l2*lc3*cos(q3));
    
    M31 = M13;
    
    M32 = M23;
    
    M33 = I3 + m3*lc3^2;
    
    % Alternative
    %M22 = I2 + I3 + m2*lc2^2 + m3*(l2^2 + lc3^2 + l2*lc3*cos(q3));

    %M12 = I2 + I3 + m2*(lc2^2 + l1*lc2*cos(q2)) + ...
    %    m3*(l2^2 + lc3^2 + l1*l2*cos(q2) + l1*lc3*cos(q2 + q3) + 2*l2*lc3*cos(q3));
    %
    
    M = [M11, M12, M13;
        M21, M22, M23;
        M31, M32, M33];
    
    %% Coriolis
    C11 = -(m2*l1*lc2*sin(q2) + m3*l1*l2*sin(q2) + m3*l1*lc3*sin(q2 + q3))*dq2 + ...
        -(m3*l1*lc3*sin(q2 + q3) + m3*l2*lc3*sin(q3))*dq3;
    
    C12 = -(m2*l1*lc2*sin(q2) + m3*l1*l2*sin(q2) + m3*l1*lc3*sin(q2 + q3))*(dq1 + dq2) + ...
        -(m3*l1*lc3*sin(q2 + q3) + m3*l2*lc3*sin(q3))*dq3;
    
    C13 = -(m3*l1*lc3*sin(q2 + q3) + m3*l2*lc3*sin(q3))*(dq1 + dq2 + dq3);
    
    C21 = (m2*l1*lc2*sin(q2) + m3*l1*l2*sin(q2) + m3*l1*lc3*sin(q2 + q3))*dq1 + ...
        -m3*l2*lc3*sin(q3)*dq3;
    
    C22 = -(m3*l2*lc3*sin(q3))*dq3;
    
    C23 = -(m3*l2*lc3*sin(q3))*(dq1 + dq2 + dq3);
    
    C31 = (m3*l1*lc3*sin(q2 + q3) + m3*l2*lc3*sin(q3))*dq1 + m3*l2*lc3*sin(q3)*dq3;
    
    C32 = m3*l2*lc3*sin(q3)*(dq1 + dq2);
    
    C33 = 0;
    
    C = [C11, C12, C13;
        C21, C22, C23;
        C31, C32, C33];
    
    % Alternative
    C11 = -((m2*l1*lc2 + m3*l1*l2)*sin(q2) + m3*l1*lc3*sin(q2 + q3))*(2*dq1*dq2 + dq2^2) + ...
        -( m3*l1*lc3*sin(q2 + q3) + m3*l2*lc3*sin(q3))*(2*dq1*dq3 + 2*dq2*dq3 + dq3^2);

    C21 = ((m2*l1*lc2 + m3*l1*l2)*sin(q2) + m3*lc3*l1*sin(q2 + q3))*(dq1^2) + ...
        - m3*lc3*l2*sin(q3)*(2*dq1*dq3 + 2*dq2*dq3 + dq3^2);
    
    C31 = (m3*l1*lc3*sin(q2 + q3) + m3*l2*lc3*sin(q3))*(dq1^2) + ...
        m3*l2*lc3*sin(q3)*(2*dq1*dq2 + dq2^2);
    
    C = [C11; C21; C31];
    %
    %% Gravity
    G11 = g*(m1*lc1 + m2*l1 + m3*l1)*cos(q1) + g*(m2*lc2 + m3*l2)*cos(q1 + q2) + g*m3*lc3*cos(q1 + q2 + q3);
    
    G21 = m2*g*lc2*cos(q1 + q2) +m3*g*(l2*cos(q1 + q2) + lc3*cos(q1 + q2 + q3));
    
    G31 = g*m3*lc3*cos(q1 + q2 + q3);
    
    G = [G11;
        G21;
        G31];
    
    % Model matrices
    [M_invd, C_invd, G_invd] = invDyn(q_m(:,i), dq_m(:,i));
    
    if(isequal(M,M_invd) == 0 || isequal(G,G_invd) == 0 || isequal(C,C_invd) == 0)
        disp('Beware.')
    end
    
    % Gravity compensator
    T_g(1:3, i) = G;
    
    % Computed-torque
    %T_d(1:3, i) = M*ddq_d(:, i) + C*dq_d(:, i) + G + F*dq_d(:,i);
    T_d(1:3, i) = 0;
    
    % Simulation (page 189, Craig)
    % ddq_m(:, 1) = M\(T_d(:, i) - C*dq_m(:, i) - G - F*dq_m(:, i));
    
    ddq_m(:, 1) = M\(T_d(:, i) - C - G - F*dq_m(:, i));
    dq_m(:, i + 1) = dq_m(:, i) + ddq_m(:, i)*dt;
    q_m(:, i + 1) = q_m(:, i) + dq_m(:, i)*dt + 0.5*ddq_m(:, i)*dt*dt;
    
%     if (q_m(1,i + 1) > 2*pi || q_m(1,i + 1) < -2*pi)
%         disp('Reseting q1.')
%         q_m(1,i + 1) = q_m(1,i + 1) - sign(q_m(1,i + 1))*2*pi;
%     end
%     
%     if (q_m(2,i + 1) > 2*pi || q_m(2,i + 1) < -2*pi)
%         disp('Reseting q2.')
%         q_m(2,i + 1) = q_m(2,i + 1) - sign(q_m(2,i + 1))*2*pi;
%     end
%     
%     if (q_m(3, i + 1) > 2*pi || q_m(3,i + 1) < -2*pi)
%         disp('Reseting q3.')
%         q_m(3, i + 1) = q_m(3, i + 1) - sign(q_m(3,i + 1))*2*pi;
%     end
    
    % Control Loop
    if (control)
        q_error(:, i) = q_d(:, i + 1) - q_m(:, i + 1);
        
        if i ~= 1
            int_error(:, i) = int_error(:, i - 1) + q_error(:, i)*dt;
            dq_error(:, i) = (q_error(:, i) - q_error(:, i - 1))*dt;
        end
        
        %T_a(:, i) = K_p * q_error(:, i) + K_d * dq_error(:, i) + K_i * int_error(:, i) + T_g(:,i);
        %T_a(3, i) = 0;
        %T_a(:,i) = [0; 0; 0];
        
        % Simulation (page 189, Craig)
        %ddq_m(:, 1) = M\(T_d(:, i) - C*dq_m(:, i) - G - F*dq_m(:,i) + T_a(:,i));
        ddq_m(:, 1) = M\(T_d(:, i) - C - G - F*dq_m(:,i) + T_a(:,i));
        dq_m(:, i + 1) = dq_m(:, i) + ddq_m(:, i)*dt;
        q_m(:, i + 1) = q_m(:, i) + dq_m(:, i)*dt + 0.5*ddq_m(:, i)*dt*dt;
        
    end
end

%% Animation
h = figure;
if(play)
    
    O = [0 0];
    A = l(1)*[cos(q_m(1,:)); sin(q_m(1,:))];
    B = A + l(2)*[cos(q_m(2,:) + q_m(1,:)); sin(q_m(2,:) + q_m(1,:))];
    C = B + l(3)*[cos(q_m(3,:) + q_m(2,:) + q_m(1,:)); sin(q_m(3,:) + q_m(2,:) + q_m(1,:))];
    
    % Plot a circle at the origin
    rc = 0.01;
    th = 0:pi/50:2*pi;
    xunit = rc * cos(th) + O(1,1);
    yunit = rc * sin(th) + O(1,2);
    
    for i = 1 : length(t)
        if(ishandle(h))
            plot([O(1,1) A(1,i) B(1,i) C(1,i)], [O(1,2) A(2,i) B(2,i) C(2,i)], 'k', xunit, yunit, 'k', 'LineWidth', 3)
            axis([-4 4 -4 4]) %axis([XMIN XMAX YMIN YMAX])
            grid on
            pause(0.0005)
        end
    end
end


%% Plots
figure
plot(t,T_d(1,:),'b',t,T_d(2,:),'r',t,T_d(3,:),'g')
title('Joint Input Torque')
legend('Joint 1','Joint 2', 'Joint 3')
xlabel('Time (s)')
ylabel('Torque (N.m)')
grid on

% figure
% plot(t,T_dist(1,:),'b',t,T_dist(2,:),'r',t,T_dist(3,:),'g')
% title('Joint Disturbance Torque')
% legend('Joint 1','Joint 2', 'Joint 3')
% xlabel('Time (s)')
% ylabel('Torque (N.m)')
% grid on

figure
plot(t,T_a(1,:),'b',t,T_a(2,:),'r',t,T_a(3,:),'g')
title('Joint Actuator Torque')
legend('Joint 1','Joint 2', 'Joint 3')
xlabel('Time (s)')
ylabel('Torque (N.m)')
grid on


figure
plot(t,q_m(1,:),'b',t,q_d(1,:),'--b',t,q_m(2,:),'r',t,q_d(2,:),'--r',t,q_m(3,:),'g',t,q_d(3,:),'--g')
title('Position')
legend('Measured Joint 1','Desired Joint 1','Measured Joint 2','Desired Joint 2',' Measured Joint 3', 'Desired Joint 3')
xlabel('Time (s)')
ylabel('Position (rad)')
grid on
axis tight

figure
plot(t,dq_m(1,:),'b',t,dq_d(1,:),'--b',t,dq_m(2,:),'r',t,dq_d(2,:),'--r',t,dq_m(3,:),'g',t,dq_d(3,:),'--g')
title('Velocity')
legend('Measured Joint 1','Desired Joint 1','Measured Joint 2','Desired Joint 2',' Measured Joint 3', 'Desired Joint 3')
xlabel('Time (s)')
ylabel('Velocity (rad/s)')
grid on
axis tight

figure
plot(t,ddq_m(1,:),'b',t,ddq_d(1,:),'--b',t,ddq_m(2,:),'r',t,ddq_d(2,:),'--r',t,ddq_m(3,:),'g',t,ddq_d(3,:),'--g')
title('Acceleration')
legend('Measured Joint 1','Desired Joint 1','Measured Joint 2','Desired Joint 2',' Measured Joint 3', 'Desired Joint 3')
xlabel('Time (s)')
ylabel('Acceleration (rad/s^2)')
grid on
axis tight
