% Control - 2 DoF Robotic Leg with SEA actuator and Fluid Environment
clear variables; close all; clc;
addpath('..\..\shared-functions')

%% Play animation?
play = 0;
isBackdrivable = 0;
isImpedance = 0;
isTaskSpace = 1;
isRigid = 1;
dampingPlots = 1;

%% Simulation parameters
[t, dt] = SimulationTime(0,10,1e-4);
dt_c = 5e-3;

%% Variables
run('TwoDOF_Variables.m')

%% Desired trajectory (MATLAB-generated)
offset = [-pi/2; 0];
A = 60*pi/180;
w = 1*pi;

[q_d, dq_d] = desiredAngularTrajectory(A, w, offset, t);

%% Desired trajectory (external data)
fileName = 'qDesired.mat';
[q_d, dq_d] = desiredAngularTrajectoryKirtley(fileName, offset, t, dt);

%% Patient joints
patientOffset = [-pi/2+pi/6; 0];

[q_p, dq_p] = patientJoints(A, w, patientOffset, t);

q_p(1,:) = q_d(1,:);
q_p(2,:) = q_d(2,:);
dq_p = 0*dq_p;

%% Cartesian coordinates (Forward Kinematics)
[x_d, y_d, ~] = L_forward(l, q_d, dof);

%% Initial conditions
q(1,1) = q_d(1,1);
dq(1,1) = 0*A*w;

q(2,1) = q_d(2,1);
dq(2,1) = 0*A*w;

%% Environment dynamics
K_env = diag([0, 0]);       %N.m
B_env = diag([50, 50]);     %N.s/m
x_wall = 0.25;              %m

%% Admittance controller gains
K_adm = 50;     %Nm/rad
B_adm = 10;     %Nms/rad
M_adm = 0;		%Nms^2/rad

%% Impedance controller gains
K_imp = 80;     %Nm/rad
B_imp = 0;      %Nms/rad
M_imp = 0;      %Nms^2/rad

%% Rotary Series Elastic Actuator (rSEA) model
K_SEA = 104;        %Nm/rad
B_SEA_vector = 30;  %Nms/rad

%% Inner torque controller
kpTorque = 380;
kiTorque = 35;
kdTorque = 0;

%% Patient model (inactive)
% Kpat = diag([200 120 60]);
% Bpat = diag([10 5 1]);

j = 1;

% Simulation (Forward Dynamics)
for B_index = 1 : length(B_SEA_vector)
    for i = 1 : length(t) - 1
        B_SEA = B_SEA_vector(B_index);
        
        % Robot Matrices
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
        
        % Control loop (200 Hz)
        if rem(i,dt_c/dt) == 0
            t_c(j) = t(i); dt = dt_c;
            
            % Interaction torque
            T_p(:,i) = K_SEA*(q(:,i) - q_p(:,i)) + B_SEA * ( dq(:,i) - dq_p(:,i) );
            
            % Environment torque
            if isRigid
                % Check if robot is touching wall
                if coord_x > x_wall
                    % Compute environment force
                    F_env(:, i) = [K_env(1,1)*(coord_x - x_wall) + B_env(1,1)*dX(1,i); 0];
                else
                    F_env(:, i) = [0; 0];
                end
                % Transform external force into torque at joints
                T_env(:, i) = Jacobian'*F_env(:, i);
                
            % Fluid environment
            else
                % Fluid torque (maybe change to hydrodynamic model)
                T_env(:, i) = K_env*(q(:,i)) + B_env*dq(:,i);
                T_env(:, i) = [0; T_env(2,i)];
            end
            
            % Compute torque error
            T_error_out(:, i) = T_d_out(:, i) - T_env(:, i);
            
            % Admittance Control Law
            if i > 2 && (K_adm ~= 0 || B_adm ~= 0 || M_adm ~= 0)
                % Trajectory correction
                q_r(:,i) = (T_error_out(:,i) + q_r(:,i - 1).*(2*M_adm/(dt^2) + B_adm/dt) - q_r(:,i - 2).*M_adm/(dt^2))./(M_adm/(dt^2) + B_adm/dt + K_adm); end
            if i > 2 && (K_adm ~= 0 || B_adm ~= 0 || M_adm ~= 0)
                % Velocity correction
                dq_r(2,i) = ( q_r(2,i) - q_r(2,i-1) )/ dt;
            end
            
            % PID Position control
            q_error(:, i) = q_d(:,i) + q_r(:,i) - q(:,i);
            dq_error(:, i) = 0*dq_d(:,i) + 0*dq_r(:,i) - dq(:,i);
            
            if i ~= 1
                int_error(:, i) = int_error(:, i - 1) + (q_error(:,i) + q_error(:, i - 1))*dt/2;
            end
            
            %Impedance torque
            T_r(:,i) = K_imp * q_error(:,i) - B_imp * dq(:,i);
            
            T_error_in(:,i) =  T_r(:, i) + T_d_in(:, i) - ( T_p(:, i) + T_env(:, i) );
            
            if i > 1
                T_error_in_int(:,i) =  T_error_in_int(:, i - 1) + (T_error_in(:,i) + T_error_in(:,i - 1))*dt/2;
                dT_error_in(:,i) = ( T_error_in(:, i ) + T_error_in(:,i - 1) ) / dt;
            end
            
            T_a(:,i) = kpTorque * T_error_in(:,i) + kiTorque * T_error_in_int(:,i) + kdTorque * dT_error_in(:,i);
            T_a(1, i) = Saturation( T_a(1, i), 2*torque_max, 2*torque_min);
            T_a(2, i) = Saturation( T_a(2, i), torque_max, torque_min);
            j = j + 1;
        else
            % Zero-Order Holder (ZOH)
            if i > 1
                F_env(:, i) = F_env(:, i - 1);
                T_env(:, i) = T_env(:, i - 1);
                T_error_out(:, i) = T_error_out(:, i - 1);
                T_error_in(:, i) = T_error_in(:, i - 1);
                T_error_in_int(:, i) = T_error_in_int(:, i - 1);
                dT_error_in(:, i) = dT_error_in(:, i - 1);
                T_a(:, i) = T_a(:, i - 1);
                T_r(:, i) = T_r(:, i - 1);
                q_r(:, i) =  q_r(:, i - 1);
                T_p(:, i) = T_p(:, i - 1);
                dq_r(:, i) = dq_r(:, i - 1);
                q_p(:, i) = q_p(:, i-1);
                dq_p(:, i) = dq_p(:, i-1);
            end
        end
        dt = t(2) - t(1);
        
        % Patient Simulation
        %q_p(:, i + 1) = ( Kpat * ( q(:,i) - q_p(:,i) ) - T_p(:,i) - T_a(2,i) ) * (dt/Bpat) + q_p(:, i);
        %dq_p(:, i + 1) = ( q_p(:, i + 1) - q_p(:, i) ) / dt;
        
        % Simulation
        ddq(:, i) = H\(T_d(:, i) + T_a(:,i) - C*dq(:, i) - G - 0*F*dq(:, i));
        dq(:, i + 1) = dq(:, i) + ddq(:, i)*dt;
        q(:, i + 1) = q(:, i) + dq(:, i)*dt + 0.5*ddq(:, i)*dt*dt;
        
        % Knee Joint
        kneeStateEstimate = (Fk*kneeState) + (Bk*input);
        Pk = Fk*(P*Fk') + Qk;
        Kk = (Pk*Hk')*inv((Hk*(Pk*Hk')) + Rk);
        % Values update
        P = Pk - Kk*(Hk*Pk);
        kneeState = kneeStateEstimate + Kk*(q(2,i) - Hk*kneeStateEstimate);
        % Variable asignment
        kalman_states(3:4,i) = kneeState;
        
        q(2, i) = kneeState(1,1);
        dq(2,i) = kneeState(2,1);
        
        % Hip Joint
        hipStateEstimate = (Fk*hipState) + (Bk*input);
        Pk = Fk*(P*Fk') + Qk;
        Kk = (Pk*Hk')*inv((Hk*(Pk*Hk')) + Rk);
        
        % Values update
        P = Pk - Kk*(Hk*Pk);
        hipState = hipStateEstimate + Kk*(q(1,i) - Hk*hipStateEstimate);
        
        % Variable asignment
        kalman_states(1:2,i) = hipState;
        
        q(1, i) = hipState(1,1);
        dq(1,i) = hipState(2,1);
        % End
    end
end

%% Joints
figure
subplot(2,1,1)
plot(t,q(1,:),'b',t,q_d(1,:),'--b')
title('Joint displacements')
legend('Measured Joint 1','Desired Joint 1','Location','Southeast')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on

subplot(2,1,2)
plot(t,q(2,:),'r',t,q_d(2,:),'--r',t,q_d(2,:)+q_r(2,:),':b',t,q_r(2,:),'-.r')
legend('Measured Joint 2','Desired Free Joint 2','Desired Corrected','Adm. Correction','Location','Southeast')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on

%% Velocities
figure
subplot(2,1,1)
plot(t,dq(1,:),'b',t,dq_d(1,:),'--b')
title('Joint velocities')
legend('Measured Joint 1','Desired Joint 1','Location','Southeast')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on

subplot(2,1,2)
plot(t,dq(2,:),'r',t,dq_d(2,:),'--r')
legend('Measured Joint 2','Desired Joint 2','Location','Southeast')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on

%% Torques
figure
plot(t,T_r(1,:),'b',t,T_r(2,:),'r')
title('Torque Reference (Virtual Impedance)')
legend('Joint 1','Joint 2')
set(gca, 'FontName', 'CMU Serif')
grid on

figure
plot(t,T_p(1,:),'b',t,T_p(2,:),'r')
title('Patient Torque')
legend('Joint 1','Joint 2')
set(gca, 'FontName', 'CMU Serif')
grid on

figure
plot(t,T_error_in(1,:),'b',t,T_error_in(2,:),'r')
title('Torque Error')
legend('Joint 1','Joint 2')
set(gca, 'FontName', 'CMU Serif')
grid on