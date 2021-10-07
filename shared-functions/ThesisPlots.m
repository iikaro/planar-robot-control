%% Thesis Plots
%% Joints displacement
% Hip Joint
figure
subplot(2,1,1);
plot(t,q_d(1,:),'-.k',t,q(1,:),'b');
%plot(t,q_d(1,:),'-.k',time,angle(:,1),'m',t,q(1,:),'b');
%legend('Desired','Real','Simulation','Orientation','Horizontal','Location','Best')
ylabel('Hip joint (rad)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
set(gca,'XMinorGrid','on')
set(gca,'MinorGridLineStyle',':')
set(gca,'GridLineStyle',':')
if length(t) > length(time)
    xlim([0 t(end)])
else
    xlim([0 time(end)])
end
%
subplot(2,1,2);
plot(t,q_d(2,:),'-.k',time,angle(:,2),'m',t,q(2,:),'b');
legend('Desired','Real','Simulation','Orientation','Horizontal','Location','Best')
ylabel('Knee joint (rad)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
set(gca,'XMinorGrid','on')
set(gca,'MinorGridLineStyle',':')
set(gca,'GridLineStyle',':')
if length(t) > length(time)
    xlim([0 t(end)])
else
    xlim([0 time(end)])
end

ylim([-1 0])

xlabel('Time (s)')
%% Joints velocities
% Hip Joint
figure
subplot(2,1,1);
%plot(t,dq(1,:),'b');
plot(t,dq_d(1,:),'-.k',time,angularVelocity(:,1),'m',t,dq(1,:),'b');
%legend('Desired','Real','Simulation','Orientation','Horizontal','Location','Best')
ylabel('Hip joint (rad/s)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
set(gca,'XMinorGrid','on')
set(gca,'MinorGridLineStyle',':')
set(gca,'GridLineStyle',':')
if length(t) > length(time)
    xlim([0 t(end)])
else
    xlim([0 time(end)])
end
%
subplot(2,1,2);
plot(time,angularVelocity(:,2),'m',t,dq(2,:),'b');
legend('Real','Simulation','Orientation','Horizontal','Location','Best')
ylabel('Knee joint (rad/s)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
set(gca,'XMinorGrid','on')
set(gca,'MinorGridLineStyle',':')
set(gca,'GridLineStyle',':')
if length(t) > length(time)
    xlim([0 t(end)])
else
    xlim([0 time(end)])
end

%ylim([-1 0])

xlabel('Time (s)')
%% Only knee joint
% Hip Joint
figure
subplot(2,1,1);
plot(t,q_d(2,:),':k',time,angle(:,2),'k',t,q(2,:),'b');
legend('Desired','Real','Simulation','Location','Southeast')
ylabel('Knee joint (rad)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
set(gca,'XMinorGrid','on')
set(gca,'YMinorGrid','on')
set(gca,'MinorGridLineStyle',':')
set(gca,'GridLineStyle',':')
if length(t) > length(time)
    xlim([0 t(end)])
else

    xlim([0 time(end)])
end
xlim([4 14])
%ylim([-1 0])
%
subplot(2,1,2);
plot(time,angularVelocity(:,2),'k',t,dq(2,:),'b');
%legend('Real','Simulation','Orientation','Horizontal','Location','Best')
ylabel('Knee joint (rad/s)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
set(gca,'XMinorGrid','on')
set(gca,'YMinorGrid','on')
set(gca,'MinorGridLineStyle',':')
set(gca,'GridLineStyle',':')
if length(t) > length(time)
    xlim([0 t(end)])
else
    xlim([0 time(end)])
end
xlabel('Time (s)')
xlim([7 18])
return
%% Only knee joint, only experiment
% Hip Joint
figure
subplot(2,1,1);
plot(time,qDesired(:,2),'-.k',time,qCorrection(:,2),'b',time,angle(:,2),'m');
legend('Desired','Admittance','Robot','Orientation','Horizontal','Location','Best')
ylabel('Knee joint (rad)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
set(gca,'XMinorGrid','on')
set(gca,'YMinorGrid','on')
set(gca,'MinorGridLineStyle',':')
set(gca,'GridLineStyle',':')
if length(t) > length(time)
    xlim([0 t(end)])
else
    xlim([0 time(end)])
end
xlim([7 17])
%ylim([-1 0])
%
subplot(2,1,2);
plot(time,angularVelocity(:,2),'m');
%legend('Real','Simulation','Orientation','Horizontal','Location','Best')
ylabel('Knee joint (rad/s)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
set(gca,'XMinorGrid','on')
set(gca,'YMinorGrid','on')
set(gca,'MinorGridLineStyle',':')
set(gca,'GridLineStyle',':')
if length(t) > length(time)
    xlim([0 t(end)])
else
    xlim([0 time(end)])
end
xlabel('Time (s)')
xlim([7 17])
return
%%
disp('Maximum absolute position and velocity error between simulated and real')
(max(abs(angle(:,2))) - max(abs(q_d(2,:))) )/ max(abs(q_d(2,:))) * 100
(max(abs(q(2,:))) - max(abs(q_d(2,:)))) / max(abs(q_d(2,:))) *100
max(abs(angularVelocity(:,2)))
max(abs(q(2,:)))
%%
disp('RMS Error between simulation and real (rad)')
rms(angle(:,2) - q(2,1:length(angularVelocity(:,2)))')
disp('RMS Error between simulation and real (rad/s)')
rms(angularVelocity(:,2) - dq(2,1:length(angularVelocity(:,2)))')

disp('RMS Error between real and desired (rad)')
rms(angle(:,2) - q_d(2,1:length(angularVelocity(:,2)))')
disp('RMS Error between real and desired (rad/s)')
rms(angularVelocity(:,2) - dq_d(2,1:length(angularVelocity(:,2)))')

disp('RMS Error between simulation and desired (rad)')
rms(q(2,1:length(angularVelocity(:,2)))' - q_d(2,1:length(angularVelocity(:,2)))')
disp('RMS Error between simulation and desired (rad/s)')
rms(dq(2,1:length(angularVelocity(:,2)))' - dq_d(2,1:length(angularVelocity(:,2)))')
%%
fig_w = 8.5;
fig_h = 8.5*1;
paper_w = fig_w;
paper_h = fig_h;

fig = gcf;
fig.PaperUnits = 'centimeters';
set(gcf, 'PaperSize', [fig_w*2+1 fig_h])
fig.PaperPosition = [0 0 paper_w*2+1 paper_h];
%print('DampingPlots.jpeg','-djpeg',['-r' num2str(600)])
%print('DampingPlots.pdf','-dpdf','-r0')
%saveas(gcf,'ModelValidation.svg')
saveas(gcf,'AdmRigid.svg')
%saveas(gcf,'ImpedanceRigid12_04.svg')
%% Robot Torque (Impedance)
figure
%subplot(2,1,1);
plot(time,desiredRobotTorque,'b',t,T_r(2,:),'m');
legend('Real','Simulation','Orientation','Horizontal','Location','Best')
ylabel('Torque (Nm)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
set(gca,'XMinorGrid','on')
set(gca,'MinorGridLineStyle',':')
set(gca,'GridLineStyle',':')
if length(t) > length(time)
    xlim([0 t(end)])
else
    xlim([0 time(end)])
end

xlabel('Time (s)')
 
% subplot(2,1,2);
% plot(time,torqueLoad,'b',t,T_p(2,:),'m');
% legend('Real','Simulation','Orientation','Horizontal','Location','Best')
% ylabel('Torque (Nm)')
% grid on
% axis tight
% set(gca, 'FontName', 'CMU Serif')
% hold on
% set(gca,'XMinorGrid','on')
% set(gca,'MinorGridLineStyle',':')
% set(gca,'GridLineStyle',':')
% if length(t) > length(time)
%     xlim([0 t(end)])
% else
%     xlim([0 time(end)])
% end
% xlabel('Time (s)')
%%
%% Only knee joint
% Hip Joint
figure
subplot(3,1,1);
%plot(time,qDesired(:,2),'-.k',time,qCorrection(:,2),'b',time,qCorrection(:,2) + qDesired(:,2),'-.b');
plot(time,qDesired(:,2),':k',time,qCorrection(:,2),'--m',t,q_v(2,:),'--b');
legend('Desired','Correction (Real)','Correction (Simulation)','Orientation','Horizontal','Location','Best')
ylabel('Knee joint (rad)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
set(gca,'XMinorGrid','on')
set(gca,'YMinorGrid','on')
set(gca,'MinorGridLineStyle',':')
set(gca,'GridLineStyle',':')
xlim([0 10])

%
subplot(3,1,2);
plot(time,qCorrection(:,2) + qDesired(:,2),'-.m',t,q_v(2,:) + q_d(2,:),'-.b');
legend('Reference (Real)','Reference (Simulation)','Orientation','Horizontal','Location','Best')
ylabel('Knee joint (rad)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
set(gca,'XMinorGrid','on')
set(gca,'YMinorGrid','on')
set(gca,'MinorGridLineStyle',':')
set(gca,'GridLineStyle',':')
axis tight
xlim([0 10])

subplot(3,1,3);
plot(time,angle(:,2),'m',t,q(2,:),'b');
legend('Real','Simulation','Orientation','Horizontal','Location','Best')
ylabel('Knee joint (rad)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
set(gca,'XMinorGrid','on')
set(gca,'YMinorGrid','on')
set(gca,'MinorGridLineStyle',':')
set(gca,'GridLineStyle',':')
xlabel('Time (s)')
axis tight
xlim([0 10])


fig_w = 8.5;
fig_h = 8.5*1.5*2;
paper_w = fig_w;
paper_h = fig_h;

fig = gcf;
fig.PaperUnits = 'centimeters';
set(gcf, 'PaperSize', [fig_w*2+1 fig_h])
fig.PaperPosition = [0 0 paper_w*2+1 paper_h];
%print('DampingPlots.jpeg','-djpeg',['-r' num2str(600)])
%print('DampingPlots.pdf','-dpdf','-r0')
saveas(gcf,'AdmResistiveMediumBd_50.svg')
return

%% Damping plots
clear dampingPlotKneeData dampingPlotKneeVelocityData
datasets = 1;
dampingPlotKneeData = q(2,:)';
dampingPlotKneeVelocityData = dq(2,:)';
clc;
clear numberOfMarkers markerValues markerCurveIndex markerCurve markerCurveVelocity;
numberOfMarkers = 101;
markerValues = linspace(0,t(end),numberOfMarkers);
markerCurveIndex = zeros(length(markerValues),1);
markerCurve = zeros(length(markerValues),datasets);
markerCurveVelocity = zeros(length(markerValues),datasets);
for j = 1 : datasets
    for i = 1 : length(markerValues)
        if j == 1
            markerCurveIndex(i) = find(t == markerValues(i));
        end
        markerCurve(i,j) = dampingPlotKneeData(markerCurveIndex(i), j);
        markerCurveVelocity (i,j) = dampingPlotKneeVelocityData(markerCurveIndex(i) , j);
    end
end
scatter(markerValues,markerCurve(:,1))
hold on
plot(t, dampingPlotKneeData(:,1))

% Trajectory
figure
subplot(2,1,1);
% Desired trajectory
plot(t,q_d(2,:),':k');
hold on
% Actual trajectory
plot(time,angle(:,2),'-k');
% Simulated trajectory
h = plot(t,dampingPlotKneeData,'k','LineWidth',0.1); %B = 30
h.Annotation.LegendInformation.IconDisplayStyle = 'off';
scatter(markerValues,markerCurve, 'v','MarkerFaceColor','w','MarkerEdgeColor','k');

% Plot properties
legend('Desired','Real','Simulated','Location','Southeast')
ylabel('Position (rad)')
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
plot(time,angularVelocity(:,2),'k');
hold on
% Simulated velocity
h = plot(t,dampingPlotKneeVelocityData,'k','LineWidth',0.3); %B = 30
h.Annotation.LegendInformation.IconDisplayStyle = 'off';
scatter(markerValues,markerCurveVelocity, 'v','MarkerFaceColor','w','MarkerEdgeColor','k');

% Plot properties
ylabel('Velocity (rad/s)')
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
fig_w = 8.5;
fig_h = 8.5*1.5;
paper_w = fig_w;
paper_h = fig_h;

fig = gcf;
fig.PaperUnits = 'centimeters';
set(gcf, 'PaperSize', [fig_w*2 fig_h])
fig.PaperPosition = [0 0 paper_w*2 paper_h];
saveas(gcf,'ModelValidationIEEE.svg')
return

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


%% Robot Torque (Impedance)
figure
%subplot(2,1,1);
plot(time,desiredRobotTorque,'b');
legend('Real','Simulation','Orientation','Horizontal','Location','Best')
ylabel('Torque (Nm)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
set(gca,'XMinorGrid','on')
set(gca,'MinorGridLineStyle',':')
set(gca,'GridLineStyle',':')
xlim([0 10])


xlabel('Time (s)')