% Robot and desired trajectories
figure
subplot(2,1,1)
plot(t,q(1,:),'b',t,q_d(1,:),'--b')
title('Joint displacements')
legend('Measured Joint 1','Desired Joint 1','Location','Best','Orientation','Horizontal')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on

subplot(2,1,2)
plot(t,q(2,:),'r',t,q_d(2,:),'--r')
legend('Measured Joint 2','Desired Joint 2','Location','Best','Orientation','Horizontal')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on

figure
subplot(3 - isImpedance*isTaskSpace,1,1)
plot(t,F_env(1,:),'b',t,F_env(2,:),'r')
title('External Force (interaction force)')
legend('X-axis','Y-axis','Location','Best','Orientation','Horizontal')
ylabel('Force (N)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on

if(isTaskSpace)
    if (isImpedance)
        subplot(3 - isImpedance,1,2)
        plot(t,T_r(1,:),'b',t,T_r(2,:),'r')
        title('Impedance Force')
        legend('X-axis','Y-axis','Location','Best','Orientation','Horizontal')
        ylabel('External Force (N)')
        grid on
        axis tight
        set(gca, 'FontName', 'CMU Serif')
        hold on
    else
        subplot(3,1,2)
        plot(t,x_r(1,:),'b',t,x_r(2,:),'r')
        title('Offset Displacement')
        legend('X-axis','Y-axis','Location','Best','Orientation','Horizontal')
        ylabel('Displacement (m)')
        grid on
        axis tight
        set(gca, 'FontName', 'CMU Serif')
        hold on
        
        subplot(3,1,3)
        plot(t,q_r(1,:),'b',t,q_r(2,:),'r')
        title('Inverse Kinematics')
        legend('Offset Joint 1','Offset Joint 2','Location','Best','Orientation','Horizontal')
        ylabel('Displacement (rad)')
        grid on
        axis tight
        set(gca, 'FontName', 'CMU Serif')
        hold on
    end
else
    subplot(3,1,2)
    plot(t,T_error(1,:),'b',t,T_error(2,:),'r')
    title('Torque Error (interaction torque)')
    legend('Joint 1','Joint 2','Location','Best','Orientation','Horizontal')
    ylabel('Torque (Nm)')
    grid on
    axis tight
    set(gca, 'FontName', 'CMU Serif')
    hold on
    
    subplot(3,1,3)
    plot(t,q_r(1,:),'b',t,q_r(2,:),'r')
    title('Angular correction offset')
    legend('Offset Joint 1','Offset Joint 2','Location','Best','Orientation','Horizontal')
    ylabel('Displacement (rad)')
    grid on
    xlabel('Time (s)')
    axis tight
    set(gca, 'FontName', 'CMU Serif')
    hold on
end

%%
figure
subplot(2,1,1)
plot(t,T_a(1,:),'b')
title('Actuator effort')
ylabel('Torque (Nm)')
grid on
set(gca, 'FontName', 'CMU Serif')
legend('Joint 1','Location','Best','Orientation','Horizontal')
hold on

subplot(2,1,2)
plot(t,T_a(2,:),'r')
ylabel('Torque (Nm)')
grid on
legend('Joint 2','Location','Best','Orientation','Horizontal')

dim = [0.3, 0.1, 0.1, 0.1];
str=sprintf('Torque RMS = %.2f Nm',rms(T_a(2,:))); %if No floting varibale number, use %d
annotation('textbox',dim,'String',str,'BackgroundColor','w','FitBoxToText','on','fontsize',10,'FontName', 'CMU Serif');
drawnow;
set(gca, 'FontName', 'CMU Serif')
hold on
%saveas(gcf,'q_m_d_q_r_f_error','svg')
%%
figure
subplot(2,1,1)
plot(t,y_d,'b',t,y,'r')
title('End-effector trajectory')
ylabel('y (m)')
grid on
set(gca, 'FontName', 'CMU Serif')
legend('Desired trajectory','Measured trajectory','Location', 'Best','Orientation','Horizontal')
hold on

subplot(2,1,2)
plot(t,x_d,'b',t,x,'r')
xlabel('Time (s)')
ylabel('x (m)')
grid on
set(gca, 'FontName', 'CMU Serif')
hold on
hline(x_wall(1),':b')
%saveas(gcf,'X_d_X_m','svg')
%%

%%
%{
figure
plot(x_d,y_d,'b',x,y,'r')
title('End-effector trajectory')
legend('Desired trajectory','Measured trajectory','Location','Northwest')
xlabel('x (m)')
ylabel('y (m)')
r = rectangle('Position',[0.5 -1 0.5 0.4]');
r.FaceColor = [.01 .01 .01 0.05];
r.EdgeColor = 'None';
r.LineStyle = 'None';
vline(0.5, ':k', 'Environment');
grid on

set(gca, 'FontName', 'CMU Serif')
saveas(gcf,'qm_qd','svg')
%}
%% Robot trajectory and external force (same plot)
%{
figure('Name','Robot trajectory and external force (same plot)')
plot(t,q_m(1,:),'b',t,q_m(2,:),'r',t,F_error(1,:),':b',t,F_error(2,:)*0.1,':r')
title('Position')
legend('Measured Joint 1','Measured Joint 2','Force Error Joint 1','Force Error Joint 2')
xlabel('Time (s)')
ylabel('Position (rad)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
saveas(gcf,'qm_Ferror','svg')
%}
%% Robot trajectory and external force (subplot style)
%{
figure('Name','Robot trajectory and external force (subplot style)')
subplot(2,1,1)
plot(t,q_m(1,:),'b',t,q_m(2,:),'r')
title('Position')
legend('Measured Joint 1','Measured Joint 2')
ylabel('Position (rad)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')

subplot(2,1,2)
plot(t,F_error(1,:),'b',t,F_error(2,:),'r')
title('Force error (interaction force)')
legend('Force Error Joint 1','Force Error Joint 2')
xlabel('Time (s)')
ylabel('Force (N)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
%}
%% Admittance control offset (rad) and sensed external force
%{
figure('Name','Admittance control offset (rad) and sensed external force')
subplot(2,1,1)
plot(t,q_r(1,:),'b',t,q_r(2,:),'r')
title('Displacement')
legend('Offset Joint 1','Offset Joint 2')
ylabel('Displacement (rad)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')

subplot(2,1,2)
plot(t,F_error(1,:),'b',t,F_error(2,:),'r')
title('Force error (interaction force)')
legend('Force Error Joint 1','Force Error Joint 2')
xlabel('Time (s)')
ylabel('Force (N)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
saveas(gcf,'qv_Ferror','svg')
%}
%% Admittance control offset (m) and sensed external force
%{
figure('Name','Admittance control offset (m) and sensed external force')
subplot(2,1,1)
plot(t,x_v(1,:),'b',t,x_v(2,:),'r')
title('Displacement')
legend('Offset Joint 1','Offset Joint 2')
ylabel('Displacement (m)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')

subplot(2,1,2)
plot(t,F_error(1,:),'b',t,F_error(2,:),'r')
title('Force error (interaction force)')
legend('Force Error Joint 1','Force Error Joint 2')
xlabel('Time (s)')
ylabel('Force (N)')
grid on
axis tight

% dim = [0.65, 0.15, 0.1, 0.1];
% maximum = max(abs(F_error(2,:)));
% average = mean(F_error(2,:));
% std_dev = std(F_error(2,:));
% delete(findall(gcf,'type','annotation'));
% str=sprintf('F_{max} = %.2f N \nF_{mean} = %.2f N \n sigma(F) = %.2f N',maximum, average, std_dev); %if No floting varibale number, use %d
% annotation('textbox',dim,'String',str,'BackgroundColor','w','FitBoxToText','on','fontsize',10,'Fontname','CMU Serif');
% drawnow;
set(gca, 'FontName', 'CMU Serif')
%}
%% Robot velocities and desired velocities
% figure('Name','Robot velocities and desired velocities')
% plot(t,dq_m(1,:),'b',t,dq_d(1,:),'--b',t,dq_m(2,:),'r',t,dq_d(2,:),'--r')
% title('Velocity')
% legend('Measured Joint 1','Desired Joint 1','Measured Joint 2','Desired Joint 2')
% xlabel('Time (s)')
% ylabel('Velocity (rad/s)')
% grid on
% axis tight
% set(gca, 'FontName', 'CMU Serif')
%% Robot accelerations and desired accelerations
% figure('Name','Robot accelerations and desired accelerations')
% plot(t,ddq_m(1,:),'b',t,ddq_d(1,:),'--b',t,ddq_m(2,:),'r',t,ddq_d(2,:),'--r')
% title('Acceleration')
% legend('Measured Joint 1','Desired Joint 1','Measured Joint 2','Desired Joint 2')
% xlabel('Time (s)')
% ylabel('Acceleration (rad/s^2)')
% grid on
% axis tight
% set(gca, 'FontName', 'CMU Serif')