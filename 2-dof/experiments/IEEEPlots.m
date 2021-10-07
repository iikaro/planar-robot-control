%% IEEE Transactions on Mechatronics Plots
clear variables; close all; clc;
addpath('D:\Users\Public\Documents\Git\ExoTaoHaptics\ExoTaoHaptics\ROBOT_TEST\SCARA\REHAB_ROBOT\Data')
addpath('G:\My Drive\Notebook\Documentos\Mestrado\github\planar-robot-control\shared-functions')

%% Hyperparameters
admittancePlots = 0;

%% Rigid wall, impedance
%data = load('Data/data_17-12-2020_12-04-59.dat'); %500
data = load('data_17-12-2020_12-08-59.dat');%1000 e 5
%data = load('Data/data_17-12-2020_12-13-16.dat');%10k e 5

%% Rigid wall, admittance
%data = load('Data/data_17-12-2020_16-03-42.dat');
%%data = load('Data/data_17-12-2020_15-09-35.dat');
if admittancePlots
    data = load('data_17-12-2020_15-17-53.dat'); %100 e 5
end
%data = load('Data/data_17-12-2020_12-16-55.dat');

%% Data import
time = data(:,1);
angle = [data(:,2), data(:,3), data(:,4)];
angularVelocity = [data(:,5), data(:,6), data(:,7)];
position = [data(:,8), data(:,9)];
velocity = [data(:,10), data(:,11)];
externalForce = [data(:,12), data(:,13)];
externalTorque = [data(:,14), data(:,15), data(:,16)];
qCorrection = [data(:,17), data(:,18), data(:,19)];
qDesired = [data(:,20), data(:,21), data(:,22)];
hapticControlSignal = data(:,23);
impedanceControlSignal = data(:,24);
kneeAngleEstimation = data(:,25);
kneeVelocityEstimation = data(:,26);
gravityCompensation = [data(:,27), data(:,28), data(:,29)];
%desiredRobotTorque = data(:,30);

%% Knee position
fig = figure;
left_color = [0.15 0.15 0.15];
right_color = [0.15 0.15 0.15];
x_lim = [15 45];
set(fig,'defaultAxesColorOrder',[left_color; right_color]);

subplot(3,1,1)
desiredPlot = plot(time, rad2deg(qDesired(:,2))',':k')
hold on
jointPlot = plot(time, rad2deg(angle(:,2)),'-k','LineWidth',1)

% Plot properties
ylabel('Knee joint (deg)')
grid on; axis tight;
set(gca, 'FontName', 'CMU Serif')
set(gca,'YMinorGrid','on')
set(gca,'XMinorGrid','on')
set(gca,'MinorGridLineStyle',':')
set(gca,'GridLineStyle',':')
xlim([5 15])
if admittancePlots; xlim([5 20]); end;

%% Knee velocity
subplot(3,1,2)
plot(time, rad2deg(angularVelocity(:,2)),'k','LineWidth',1)

% Plot properties
ylabel('Knee joint (deg/s)')
grid on; axis tight;
set(gca, 'FontName', 'CMU Serif')
set(gca,'YMinorGrid','on')
set(gca,'XMinorGrid','on')
set(gca,'MinorGridLineStyle',':')
set(gca,'GridLineStyle',':')
xlim([5 15])
if admittancePlots; xlim([5 20]); end;
xlabel('Time (s)')

%% End-effector
subplot(3,1,3)
plot(time, position(:,1),'k','LineWidth',1)
hold on
hline(0.25,'-k')

% Plot properties
ylabel('X Position (m)')
grid on; axis tight;
set(gca, 'FontName', 'CMU Serif')
set(gca,'YMinorGrid','on')
set(gca,'XMinorGrid','on')
set(gca,'MinorGridLineStyle',':')
set(gca,'GridLineStyle',':')
xlim([5 15])
yyaxis right
forcePlot = plot(time, externalForce(:,1),'--k','LineWidth',1)
ylabel('External force (N)')
if admittancePlots; xlim([5 20]); 
    legend([desiredPlot, jointPlot, forcePlot],'Free','Cascade','External Force','Location','Best')
end
legend([desiredPlot, jointPlot, forcePlot],'Free','Impedance','External Force','Location','Best')
%ylim([-0.1 45])
%% Save plots
fig_w = 8.5;
fig_h = 8.5*2;
paper_w = fig_w;
paper_h = fig_h;
fig = gcf;
fig.PaperUnits = 'centimeters';
set(gcf, 'PaperSize', [fig_w*2 fig_h])
fig.PaperPosition = [0 0 paper_w*2 paper_h];

if admittancePlots
    saveas(gcf,'AdmittanceRigidWall.svg')
else
    saveas(gcf,'ImpedanceRigidWall.svg')
end

%% Data analysis
maxPenetration = max(position(position(:,1) > 0.25) - 0.25)
meanPenetration = mean(position(position(:,1) > 0.25) - 0.25)
stdPenetration = std(position(position(:,1) > 0.25) - 0.25)