%%  ExoTaoHaptics - Haptic Interface for Lower Limbs Exoskeleton
%   Author: Icaro Ostan
%   Date: 30/11/2020
%% Data Analysis
clear variables; close all; clc;
updateFigures = 0;
set(0,'DefaultFigureVisible','on')

% Resistive, admittance
data = load('data_16-12-2020_16-05-11.dat'); % IEEE perfeito, bimp = 0
dataImpedance = load('data_17-12-2020_10-39-19.dat'); %*****IEEE benv = 50 somado

%% Imported variables

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
%torqueLoad = data(:,31);
%gearMotorAngle = data(:,32);
%loadVelocity = data(:,33);
%estimatedLoadVelocity = data(:,34);
%%
timeImpedance =  dataImpedance(:,1);
angleImpedance =  [dataImpedance(:,2), dataImpedance(:,3), dataImpedance(:,4)];
angularVelocityImpedance =  [dataImpedance(:,5), dataImpedance(:,6), dataImpedance(:,7)];
positionImpedance =  [dataImpedance(:,8), dataImpedance(:,9)];
velocityImpedance =  [dataImpedance(:,10), dataImpedance(:,11)];
externalForceImpedance =  [dataImpedance(:,12), dataImpedance(:,13)];
externalTorqueImpedance =  [dataImpedance(:,14), dataImpedance(:,15), dataImpedance(:,16)];
qCorrectionImpedance =  [dataImpedance(:,17), dataImpedance(:,18), dataImpedance(:,19)];
qDesiredImpedance =  [dataImpedance(:,20), dataImpedance(:,21), dataImpedance(:,22)];
hapticControlSignalImpedance =  dataImpedance(:,23);
impedanceControlSignalImpedance =  dataImpedance(:,24);
kneeAngleEstimationImpedance =  dataImpedance(:,25);
kneeVelocityEstimationImpedance =  dataImpedance(:,26);
gravityCompensationImpedance =  [dataImpedance(:,27), dataImpedance(:,28), dataImpedance(:,29)];

%% Local variables
dt = 5e-3;
timeLength = length(time);

%% IEEE Mechatronics Plots
    figure
    subplot(2,1,1)
    plot(time, rad2deg(qDesired(:,2))',':k')
    hold on
    
    plot(time, rad2deg(angle(:,2)),'-k','LineWidth',1)
    plot(time, rad2deg(qDesired(:,2) + qCorrection(:,2)),'--k')
    plot(timeImpedance, rad2deg(angleImpedance(:,2)),'-.k','LineWidth',1)
    
    legend('Free','Cascade','Cascade desired','Impedance','Location','Best')
    
    % Plot properties
    ylabel('Position (deg)')
    grid on; axis tight;
    set(gca, 'FontName', 'CMU Serif')
    set(gca,'YMinorGrid','on')
    set(gca,'XMinorGrid','on')
    set(gca,'MinorGridLineStyle',':')
    set(gca,'GridLineStyle',':')
    xlim([4 11])
    
    subplot(2,1,2)
    plot(time, rad2deg(angularVelocity(:,2)),'k','LineWidth',1)
    hold on
    plot(time, rad2deg(angularVelocityImpedance(:,2)),'-.k','LineWidth',1)
    
    % Plot properties
    ylabel('Velocity (deg/s)')
    grid on; axis tight;
    set(gca, 'FontName', 'CMU Serif')
    set(gca,'YMinorGrid','on')
    set(gca,'XMinorGrid','on')
    set(gca,'MinorGridLineStyle',':')
    set(gca,'GridLineStyle',':')
    xlim([4 11])
    xlabel('Time (s)')
    fig_w = 9*2;
    fig_h = 9*1.5;
    paper_w = fig_w;
    paper_h = fig_h;
    fig = gcf;
    fig.PaperUnits = 'centimeters';
    set(gcf, 'PaperSize', [fig_w fig_h])
    fig.PaperPosition = [0 0 paper_w paper_h];
    saveas(gcf,'ResistiveEnvironmentsPlotsIEEE.svg')
    %% Statistics Cascade
    fprintf('\Admittance Controller\n')
    qd = rad2deg(qDesired(400:end,2) + qCorrection(400:end,2));
    q = rad2deg(angle(400:end,2));
    originalPeak = max(abs(rad2deg(qDesired(400:end,2))));
    actualPeak = max(abs(rad2deg(angle(400:end,2))));
    originalRMS = rms(rad2deg(qDesired(400:end,2)));
    actualRMS = rms(q);
    error = 100*(qd - q)./qd;
    % figure
    % plot(q,':')
    % hold on
    % plot(qd,':')
    % yyaxis right
    % plot(error)
    fprintf('Mean error (deg) %f.',(mean(abs(qd-q))))
    fprintf('\nSTD error (deg) %f.',(std(abs(qd-q))))
    fprintf('\nMax error (deg) %f.',(max(abs(qd-q))))
    fprintf('\nMax error (perc) %f.' ,max(error))
    fprintf('\nError rms (deg) %f.', rms(abs(qd-q)))
    fprintf('\nRMS reduction (deg) %f.', 100*(originalRMS - actualRMS)/originalRMS )
    fprintf('\nPeak reduction (deg) %f.\n', 100*(originalPeak - actualPeak)/originalPeak )
    
    %% Statistics Impedance
    fprintf('\nImpedance Controller\n')
    qd = rad2deg(qDesired(400:end,2) + qCorrection(400:end,2));
    q = rad2deg(angleImpedance(400:end,2));
    originalPeak = max(abs(rad2deg(qDesired(400:end,2))));
    actualPeak = max(abs(rad2deg(angleImpedance(400:end,2))));
    originalRMS = rms(rad2deg(qDesired(400:end,2)));
    actualRMS = rms(q);
    error = 100*(qd - q)./qd;
    % figure
    % plot(q,':')
    % hold on
    % plot(qd,':')
    % yyaxis right
    % plot(error)
    fprintf('Mean error (deg) %f.',(mean(abs(qd-q))))
    fprintf('\nSTD error (deg) %f.',(std(abs(qd-q))))
    fprintf('\nMax error (deg) %f.',(max(abs(qd-q))))
    fprintf('\nMax error (perc) %f.' ,max(error))
    fprintf('\nError rms (deg) %f.', rms(abs(qd-q)))
    fprintf('\nRMS reduction (deg) %f.', 100*(originalRMS - actualRMS)/originalRMS )
    fprintf('\nPeak reduction (deg) %f.\n', 100*(originalPeak - actualPeak)/originalPeak )

%% Controller effort Cascade
T = time(end) + dt;
dv = abs ( max(impedanceControlSignal) - min(impedanceControlSignal) );
Q_K = 0;
u_two_norm = 0;
for i = 2 : length(time)
    Q_K = Q_K + abs( impedanceControlSignal(i) - impedanceControlSignal(i-1) );
    u_two_norm = u_two_norm + ( impedanceControlSignal(i) * impedanceControlSignal(i) + impedanceControlSignal(i - 1) * impedanceControlSignal(i - 1) ) * dt / 2;
end
Q_K_u(1) = Q_K * (1 / T) * (1 / dv);
Q(1) = u_two_norm;

%% Controller effort Impedance
T = time(end) + dt;
dv = abs ( max(impedanceControlSignalImpedance) - min(impedanceControlSignalImpedance) );
Q_K = 0;
u_two_norm = 0;
for i = 2 : length(time)
    Q_K = Q_K + abs( impedanceControlSignalImpedance(i) - impedanceControlSignalImpedance(i-1) );
    u_two_norm = u_two_norm + ( impedanceControlSignalImpedance(i) * impedanceControlSignalImpedance(i) + impedanceControlSignalImpedance(i - 1) * impedanceControlSignalImpedance(i - 1) ) * dt / 2;
end
Q_K_u(2) = Q_K * (1 / T) * (1 / dv);
Q(2) = u_two_norm;

%% Controller effort Impedance Free
data = load('data_16-12-2020_15-46-39.dat');
dt = 5e-3;
time = data(:,1);
timeLength = length(time);
impedanceControlSignal =  data(:,24);
T = time(end) + dt;
dv = abs ( max(impedanceControlSignal) - min(impedanceControlSignal) );
Q_K = 0;
u_two_norm = 0;
for i = 2 : length(time)
    Q_K = Q_K + abs( impedanceControlSignal(i) - impedanceControlSignal(i-1) );
    u_two_norm = u_two_norm + ( impedanceControlSignal(i) * impedanceControlSignal(i) + impedanceControlSignal(i - 1) * impedanceControlSignal(i - 1) ) * dt / 2;
end
Q_K_u(3) = Q_K * (1 / T) * (1 / dv);
Q(3) = u_two_norm;
return
%%
Q_K_u
Q = Q./Q(3)