%% ExoTaoHaptics Data Analysis

%% Initialization
clear variables; close all; clc;
updateFigures = 0;

%% Import Variables
qDesired(:,1) = load('hip.dat');
qDesired(:,2) = -1 * load('knee.dat');
qDesired(:,3) = load('ankle.dat');
stepLength = 459;
qDesired = rad2deg(qDesired(1:stepLength,:));

%% Plot
gaitPercentage = linspace(0,1,stepLength);
figure
plot(   gaitPercentage, qDesired(:,1),'--k',...
        gaitPercentage, qDesired(:,2),'-k', ...
        gaitPercentage, qDesired(:,3),':k');
grid on
grid minor
xlabel('Gait cycle (%)')
ylabel('Position (deg)')
legend('Hip','Knee','Ankle','Location','SouthWest')
set(gca, 'FontName', 'CMU Serif')

%% Save Figure
figName = 'DesiredOrFreeTrajectories';
figFormat = 'svg';
figWidth = 16;
figHeight = 9;
paperWidth = figWidth;
paperHeight = figHeight;
%set(gca, 'FontSize', 8)
fig = gcf;
fig.PaperUnits = 'centimeters';
set(gcf, 'PaperSize', [figWidth figHeight])
fig.PaperPosition = [0 0 paperWidth paperHeight];
saveas(gcf, figName, figFormat)

if(updateFigures)
    saveas(gcf,'DesiredTrajectories','png')
end