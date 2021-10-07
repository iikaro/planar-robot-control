% Control - 2 DoF Robotic Leg with SEA actuator and Fluid Environment
clear variables; close all; clc;
addpath('..\..\..\..\shared-functions')

% Imported variables
%data = load('data_16-12-2020_10-52-13.dat');
data = load('data_16-12-2020_15-46-39.dat');
time = data(:,1);
angle = [data(:,2), data(:,3), data(:,4)];
angularVelocity = [data(:,5), data(:,6), data(:,7)];
% Local variables
dt = 5e-3;
timeLength = length(time);

%% Play animation?
play = 0;
isBackdrivable = 0;
isImpedance = 0;
isTaskSpace = 1;
isRigid = 1;

dampingPlots = 1;
%% Simulation parameters
[t, dt] = SimulationTime(0,12,1e-4);
dt_c = 5e-3;

%% Variables
run('TwoDOF_Variables.m')

%% Desired trajectory (MATLAB-generated)
offset = [-pi/2; 0];
A = 60*pi/180;
w = 1*pi;

% First joint
q_d(1,:) = 0*A*sin(w*t) + offset(1);
dq_d(1,:) = 0*w*A*cos(w*t);

% Second joint
q_d(2,:) = A*sin(w*t) + offset(2);
dq_d(2,:) = 0*w*A*cos(w*t);

for i = 1 : length(t)
    if t(i) >= 2.5, q_d(2,i) = q_d(2,i-1); q_d(1,i) = q_d(1,i-1); end
end

%% Desired trajectory (external data)
load('qDesired.mat')
sample = 2;

for i = 1 : length(t)
    if rem(i,0.005/dt) == 0
        if sample < 461
            q_d(1,i) = qDesired(sample,1);
            dq_d(1,i) = 0;
            q_d(2,i) = qDesired(sample,2);
            dq_d(2,i) = 0;
            sample = sample + 1;
        else
            sample = 2;
            q_d(1,i) = qDesired(sample,1);
            dq_d(1,i) = 0;
            q_d(2,i) = qDesired(sample,2);
            dq_d(2,i) = 0;
            sample = sample + 1;
        end
        
    else
        if i > 1
            q_d(1,i) = q_d(1,i-1);
            q_d(2,i) = q_d(2,i-1) ;
        end
    end
end

q_d(:,1:50) = 0;
q_d(1,:) = q_d(1,:) - pi/2;

%% Patient joints
q_p = zeros(2, length(t));
dq_p = zeros(2, length(t));

% First joint
q_p(1,:) = 0*A*sin(w*t) + offset(1) + pi/6;
dq_p(1,:) = 0*w*A*cos(w*t);

% Second joint
q_p(2,:) = 0*A*sin(w*t) + offset(2);
dq_p(2,:) = 0*w*A*cos(w*t);

q_p(1,:) = q_d(1,:);
q_p(2,:) = q_d(2,:);

%% Cartesian coordinates (Forward Kinematics)
x_d = l(1)*cos(q_d(1,:)) + l(2)*cos(q_d(1,:) + q_d(2,:));
y_d = l(1)*sin(q_d(1,:)) + l(2)*sin(q_d(1,:) + q_d(2,:));

%% Initial conditions
q(1,1) = q_d(1,1);
dq(1,1) = 0*A*w;

q(2,1) = q_d(2,1);
dq(2,1) = 0*A*w;

%% Environment dynamics
K_env = diag([0, 0]);
B_env = 0*diag([50, 50]);  %N.s/m
x_wall = 0.25;  %m

%% Admittance controller gains
K_adm = 0*50;     %Nm/rad
B_adm = 0*10;     %Nms/rad
M_adm = 0;		%Nms^2/rad

%% Impedance controller gains
K_imp = 80;     %Nm/rad
B_imp = 10;      %Nms/rad
M_imp = 0;      %Nms^2/rad

%% Rotary Series Elastic Actuator (rSEA) model
K_SEA = 104;        %Nm/rad
B_SEA_vector = 30;  %Nms/rad
%B_SEA_vector = [15, 30, 45, 60];
%% Patient model (inactive)
% Kpat = diag([200 120 60]);
% Bpat = diag([10 5 1]);

%% Auxiliary variables
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
            
            T_a(:,i) = 380 * T_error_in(:,i) + 35 * T_error_in_int(:,i) + 0 * dT_error_in(:,i);
            T_a(1, i) = Saturation( T_a(1, i), 2*torque_max, 2*torque_min);
            T_a(2, i) = Saturation( T_a(2, i), torque_max, torque_min);
            j = j + 1;
        else
            % Zero-Order Holder (ZOH)
            if i > 1
                F_env(:,i) = F_env(:, i -1);
                T_env(:, i) = T_env(:, i - 1);
                T_error_out(:, i) = T_error_out(:, i - 1);
                T_error_in(:,i) = T_error_in(:,i-1);
                T_error_in_int(:,i) = T_error_in_int(:,i-1);
                dT_error_in(:,i) = dT_error_in(:,i-1);
                T_a(: , i) = T_a(:, i - 1);
                T_r(: , i) = T_r(:, i - 1);
                q_r(:,i) =  q_r(:,i - 1);
                T_p(:,i) = T_p(:,i - 1);
                dq_r(:,i) = dq_r(:,i - 1);
                q_p(:,i) = q_p(:,i-1);
                dq_p(:,i) = dq_p(:,i-1);
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
        %%
        
        % Animation
        %if(play)
        %    run('Animation_TwoDOF_Underwater.m')
        %end
        %run('DataAnalysis.m')
        
        % Plots
        %run('TwoDOF_Plots.m')
        % Plots
        %run('TwoDOF_Plots.m')
        % Plots
        %run('ThesisPlots.m')
        
        %run('DampingPlots.m')
        dampingPlotKneeData(:,B_index) = rad2deg(q(2,:));
        dampingPlotKneeVelocityData(:,B_index) = rad2deg(dq(2,:));
        
end

return
%% Damping plots
clc;
clear numberOfMarkers markerValues markerCurveIndex markerCurve markerCurveVelocity;
numberOfMarkers = 51;
markerValues = linspace(0,t(end),numberOfMarkers);
markerCurveIndex = zeros(length(markerValues),1);
markerCurve = zeros(length(markerValues),4);
markerCurveVelocity = zeros(length(markerValues),4);
for j = 1 : 4
    for i = 1 : length(markerValues)
        if j == 1
            markerCurveIndex(i) = find(t == markerValues(i));
        end
        markerCurve(i,j) = dampingPlotKneeData(markerCurveIndex(i), j);
        markerCurveVelocity (i,j) = dampingPlotKneeVelocityData(markerCurveIndex(i) , j);
    end
end
% scatter(markerValues,markerCurve(:,1))
% hold on
% plot(t, dampingPlotKneeData(:,1))

if (dampingPlots)
    % Trajectory
    figure
    subplot(2,1,1);
    % Desired trajectory
    plot(t,rad2deg(q_d(2,:)),':k');
    hold on
    % Actual trajectory
    plot(time,rad2deg(angle(:,2)),'-k');
    % Simulated trajectory
    h = plot(t,dampingPlotKneeData(:,1),'k','LineWidth',0.1); %B = 15
    h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    scatter(markerValues,markerCurve(:,1), 'o','MarkerFaceColor','w','MarkerEdgeColor','k');
    
    h = plot(t,dampingPlotKneeData(:,2),'k','LineWidth',0.1); %B = 30
    h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    scatter(markerValues,markerCurve(:,2), 'v','MarkerFaceColor','w','MarkerEdgeColor','k');
    
    h = plot(t,dampingPlotKneeData(:,3),'k','LineWidth',0.1); %B = 45
    h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    scatter(markerValues,markerCurve(:,3),'s','MarkerFaceColor','w','MarkerEdgeColor','k');
    
    h = plot(t,dampingPlotKneeData(:,4),'k','LineWidth',0.1); %B = 60
    h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    scatter(markerValues,markerCurve(:,4),'*','MarkerFaceColor','w','MarkerEdgeColor','k');
    % Plot properties
    legend('Desired','Real','15 Nms/rad','30 Nms/rad','45 Nms/rad','60 Nms/rad','Location','SouthEast')
    ylabel('Position (deg)')
    grid on; axis tight;
    set(gca, 'FontName', 'CMU Serif')
    set(gca,'YMinorGrid','on')
    set(gca,'XMinorGrid','on')
    set(gca,'MinorGridLineStyle',':')
    set(gca,'GridLineStyle',':')
    if length(t) > length(time)
        xlim([0 t(end)])
    else
        xlim([0 time(end)])
    end
    xlim([1 7])
    
    % Velocity
    subplot(2,1,2);
    % Actual velocity
    plot(time,rad2deg(angularVelocity(:,2)),'k');
    hold on
    % Simulated velocity
    h = plot(t,dampingPlotKneeVelocityData(:,1),'k','LineWidth',0.3); %B = 15
    h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    scatter(markerValues,markerCurveVelocity(:,1), 'o','MarkerFaceColor','w','MarkerEdgeColor','k');
    
    h = plot(t,dampingPlotKneeVelocityData(:,2),'k','LineWidth',0.3); %B = 30
    h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    scatter(markerValues,markerCurveVelocity(:,2), 'v','MarkerFaceColor','w','MarkerEdgeColor','k');
    
    h = plot(t,dampingPlotKneeVelocityData(:,3),'k','LineWidth',0.3); %B = 45
    h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    scatter(markerValues,markerCurveVelocity(:,3),'s','MarkerFaceColor','w','MarkerEdgeColor','k');
    
    h = plot(t,dampingPlotKneeVelocityData(:,4),'k','LineWidth',0.3); %B = 60
    h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    scatter(markerValues,markerCurveVelocity(:,4),'*','MarkerFaceColor','w','MarkerEdgeColor','k');
    
    % Plot properties
    ylabel('Velocity (deg/s)')
    grid on; axis tight;
    set(gca, 'FontName', 'CMU Serif')
    set(gca,'YMinorGrid','on')
    set(gca,'XMinorGrid','on')
    set(gca,'MinorGridLineStyle',':')
    set(gca,'GridLineStyle',':')
    if length(t) > length(time)
        xlim([0 t(end)])
    else
        xlim([0 time(end)])
    end
    xlim([1 7])
    xlabel('Time (s)')
end


%%
fig_w = 9*2;
fig_h = 9*1.5;
paper_w = fig_w;
paper_h = fig_h;
fig = gcf;
fig.PaperUnits = 'centimeters';
set(gcf, 'PaperSize', [fig_w fig_h])
fig.PaperPosition = [0 0 paper_w paper_h];
saveas(gcf,'DampingPlotsIEEE.svg')

%%
if (dampingPlots)
    % Trajectory
    figure
    subplot(2,1,1);
    % Desired trajectory
    plot(t,rad2deg(q_d(2,:)),':k');
    hold on
    % Actual trajectory
    plot(time,rad2deg(angle(:,2)),'-k');
    % Simulated trajectory
    h = plot(t,dampingPlotKneeData(:,2),'k','LineWidth',0.1); %B = 30
    h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    scatter(markerValues,markerCurve(:,2), 'v','MarkerFaceColor','w','MarkerEdgeColor','k');
    
%     h = plot(t,dampingPlotelocityData(:,1),'k','LineWidth',0.3); %B = 15
%     h.Annotation.LegendInformation.IconDisplayStyle = 'off';
%     scatter(markerValues,markerCurve(:,1), 'o','MarkerFaceColor','w','MarkerEdgeColor','k');
    
    % Plot properties
    legend('Desired','Real','Simulation','Location','SouthEast')
    ylabel('Position (deg)')
    grid on; axis tight;
    set(gca, 'FontName', 'CMU Serif')
    set(gca,'YMinorGrid','on')
    set(gca,'XMinorGrid','on')
    set(gca,'MinorGridLineStyle',':')
    set(gca,'GridLineStyle',':')
    if length(t) > length(time)
        xlim([0 t(end)])
    else
        xlim([0 time(end)])
    end
    xlim([2 12])
    
    % Velocity
    subplot(2,1,2);
    % Actual velocity
    plot(time,rad2deg(angularVelocity(:,2)),'k');
    hold on
    % Simulated velocity
    h = plot(t,dampingPlotKneeVelocityData(:,2),'k','LineWidth',0.3); %B = 30
    h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    scatter(markerValues,markerCurveVelocity(:,2), 'v','MarkerFaceColor','w','MarkerEdgeColor','k');
%     h = plot(t,dampingPlotKneeVelocityData(:,1),'k','LineWidth',0.3); %B = 15
%     h.Annotation.LegendInformation.IconDisplayStyle = 'off';
%     scatter(markerValues,markerCurveVelocity(:,1), 'o','MarkerFaceColor','w','MarkerEdgeColor','k');
    % Plot properties
    ylabel('Velocity (deg/s)')
    grid on; axis tight;
    set(gca, 'FontName', 'CMU Serif')
    set(gca,'YMinorGrid','on')
    set(gca,'XMinorGrid','on')
    set(gca,'MinorGridLineStyle',':')
    set(gca,'GridLineStyle',':')
    if length(t) > length(time)
        xlim([0 t(end)])
    else
        xlim([0 time(end)])
    end
    xlim([2 12])
    xlabel('Time (s)')
end


%%
fig_w = 9*2;
fig_h = 9*1.5;
paper_w = fig_w;
paper_h = fig_h;
fig = gcf;
fig.PaperUnits = 'centimeters';
set(gcf, 'PaperSize', [fig_w fig_h])
fig.PaperPosition = [0 0 paper_w paper_h];
saveas(gcf,'ModelValidationIEEE.svg')
return
%%
figure
plot(t,rad2deg(q(2,:)),'k')
hold on
plot(t,rad2deg(q_d(2,:)),':k')
grid minor
set(gca, 'FontName', 'CMU Serif')

%%
q = rad2deg(q(2,:));
qd = rad2deg(q_d(2,:));
%%
originalPeak = max(abs(qd));
actualPeak = max(abs(q));
originalRMS = rms(qd);
actualRMS = rms(q);
error = 100*(qd(200:end) - q(200:end))./qd(200:end);
    
fprintf('Mean error (deg) %f.',(mean(abs(qd-q))))
fprintf('\nSTD error (deg) %f.',(std(abs(qd-q))))
fprintf('\nMax error (deg) %f.',(max(abs(qd-q))))
fprintf('\nMax error (perc) %f.' ,max(error(2:end)))
fprintf('\nError rms (deg) %f.', rms(abs(qd-q)))
fprintf('\nRMS reduction (deg) %f.', 100*(originalRMS - actualRMS)/originalRMS )
fprintf('\nPeak reduction (perc) %f.\n', 100*(originalPeak - actualPeak)/originalPeak )