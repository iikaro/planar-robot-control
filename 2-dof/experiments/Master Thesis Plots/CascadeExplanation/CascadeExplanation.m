%%  ExoTaoHaptics - Haptic Interface for Lower Limbs Exoskeleton
%   Author: Icaro Ostan
%   Date: 30/11/2020
%% Data Analysis
%clear variables; close all; clc;
updateFigures = 0;
set(0,'DefaultFigureVisible','on')
data = load('data_16-12-2020_16-05-11.dat'); % IEEE perfeito, bimp = 0

%% Imported variables

time = data(:,1);
angle = [data(:,2), data(:,3), data(:,4)];
angularVelocity = [data(:,5), data(:,6), data(:,7)];
qCorrection = [data(:,17), data(:,18), data(:,19)];
qDesired = [data(:,20), data(:,21), data(:,22)];
%desiredRobotTorque = data(:,30);
%torqueLoad = data(:,31);
%gearMotorAngle = data(:,32);
%loadVelocity = data(:,33);
%estimatedLoadVelocity = data(:,34);

%% Local variables
dt = 5e-3;
timeLength = length(time);
acc(1) = 0;
for i = 2 : length(angularVelocity(:,2))
    acc(i) = ( angularVelocity(i,2) - angularVelocity(i-1,2) ) / 0.05;
end
%% Resistive Environment/Cascade explanation
figure
% subplot(2,1,1)
plot(time, rad2deg(qDesired(:,2)'),':k')
hold on
plot(time, rad2deg(qCorrection(:,2)),'-.k','LineWidth',1)
plot(time, rad2deg(qDesired(:,2) + qCorrection(:,2)),'--k')
plot(time, rad2deg(angle(:,2)),'-k','LineWidth',1)
yyaxis right
%plot(time, rad2deg(angularVelocity(:,2)))
plot(time, acc)
legend('Free','Correction','Desired','Final','Location','SouthEast')

% Plot properties
ylabel('Position (deg)')
grid on; axis tight;
set(gca, 'FontName', 'CMU Serif')
set(gca,'YMinorGrid','on')
set(gca,'XMinorGrid','on')
set(gca,'MinorGridLineStyle',':')
set(gca,'GridLineStyle',':')
xlim([4 16])

xlabel('Time (s)')
fig_w = 9*2;
fig_h = 9*0.75;
paper_w = fig_w;
paper_h = fig_h;
fig = gcf;
fig.PaperUnits = 'centimeters';
set(gcf, 'PaperSize', [fig_w fig_h])
fig.PaperPosition = [0 0 paper_w paper_h];
saveas(gcf,'CascadeExplanationIEEE.svg')
return
%% Statistics
clc
qd = rad2deg(qDesired(400:end,2) + qCorrection(400:end,2));
q = rad2deg(angle(400:end,2));
originalPeak = max(abs(rad2deg(qDesired(400:end,2))));
actualPeak = max(abs(rad2deg(angle(400:end,2))));
originalRMS = rms(rad2deg(qDesired(400:end,2)));
actualRMS = rms(q);
error = 100*(qd - q)./qd;
figure
plot(q,':')
hold on
plot(qd,':')
yyaxis right
plot(error)
fprintf('Mean error (deg) %f.',(mean(abs(qd-q))))
fprintf('\nSTD error (deg) %f.',(std(abs(qd-q))))
fprintf('\nMax error (deg) %f.',(max(abs(qd-q))))
fprintf('\nMax error (perc) %f.' ,max(error))
fprintf('\nError rms (deg) %f.', rms(abs(qd-q)))
fprintf('\nRMS reduction (deg) %f.', 100*(originalRMS - actualRMS)/originalRMS )
fprintf('\nPeak reduction (deg) %f.\n', 100*(originalPeak - actualPeak)/originalPeak )
