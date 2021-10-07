%% ExoTaoHaptics Data Analysis
% Impedance Controller

%% Initialization
clear variables; %close all; clc;
addpath('ExoData');

%% Plot configuration
alphaValue = 0.15;
fig_w = 16;
fig_h = 8;
paper_w = fig_w;
paper_h = fig_h;
figFormat = 'svg';
%% Data
subjectsTotal = 10;

testData(:,:,1) = load('data_22-02-2021_15-56-17.dat');
testData(:,:,2) = load('data_23-02-2021_16-02-28.dat');
testData(:,:,3) = load('data_24-02-2021_16-29-55.dat');
testData(:,:,4) = load('data_25-02-2021_16-05-16.dat');
testData(:,:,5) = load('data_26-02-2021_11-16-54.dat');
testData(:,:,6) = load('data_26-02-2021_20-31-20.dat');
testData(:,:,7) = load('data_04-03-2021_10-02-56.dat');
testData(:,:,8) = load('data_04-03-2021_16-00-36.dat');
testData(:,:,9) = load('data_04-03-2021_21-08-46.dat');
testData(:,:,10) = load('data_05-03-2021_11-06-59.dat');

%% Steps to be discarded from analysis
discardSteps = zeros(subjectsTotal,60);

discardSteps(1,1:4) = 1:4;
discardSteps(2,1:2) = 1:2;
discardSteps(3,1:2) = 1:2;
discardSteps(4,1:3) = 1:3;
discardSteps(5,1:4) = 1:4;
discardSteps(6,1:5) = [1:3 50:51];
discardSteps(7,1:12) = 1:12;
discardSteps(8,1:23) = [1:3, 32:51];
discardSteps(9,1:6) = 1:6;
discardSteps(10,1:4) = [1:2 50:51];
discardSteps(10,21:39) = [21, 22, 0*(23:29), 30, 31, 0*(32:36), 37, 0, 39];

%% Time data
time = testData(:,1,1);
dt = 5e-3;
timeLength = length(time);
%%
Q_K_subject = [];
u_two_norm_subjects = [];
for subject = 1 : subjectsTotal
    % Joint trajectories
    hipTrajectory = testData(:,2,subject);
    kneeTrajectory = testData(:,3,subject);
    ankleTrajectory = testData(:,4,subject);
    
    % Joint velocities
    hipVelocity = testData(:,5,subject);
    kneeVelocity = testData(:,6,subject);
    ankleVelocity = testData(:,7,subject);
    
    % Reference torque and interaction torque
    desiredTorque = testData(:,24,subject);
    loadTorque = testData(:,31,subject);
    
    % Gets the desired knee trajectory
    desiredKneeTrajectory = testData(:,21,subject);
    
    % Knee trajectory error
    kneeError = testData(:,32,subject) - testData(:,3,subject);
    
    % Constant step length
    stepLength = 459;
    firstStep = 460;
    
    % Computes total steps based on step length and trajectory length
    totalSteps = round(length (kneeTrajectory) / stepLength);
    
    % Constant x-axis
    gait = linspace(0,1,stepLength+1);
    j = 1;
    
    % Creates vector with each step trajectory
    for i = 0 : totalSteps - 1
        if ~ismember(i,discardSteps(subject,:))
            
            T = stepLength*dt + dt;
            dv = abs (max(desiredTorque(i*firstStep + 1: i*firstStep + 1 + stepLength, 1)) - min(desiredTorque(i*firstStep + 1: i*firstStep + 1 + stepLength, 1)));
            Q_K(j) = 0;
            u_two_norm(j) = 0;
            
            for timeStep = i*firstStep + 1 : i*firstStep + 1 + stepLength
                Q_K(j) = Q_K(j) + abs( desiredTorque(timeStep) - desiredTorque(timeStep-1) );
                u_two_norm(j) = u_two_norm(j) + ( desiredTorque(timeStep)  * desiredTorque(timeStep)  + desiredTorque(timeStep - 1)  * desiredTorque(timeStep - 1)  ) * dt / 2;
            end
            Q_K(j) = Q_K(j) * (1 / T) * (1 / dv);
            
            
            hipSteps(:, j) = hipTrajectory(i*firstStep + 1: i*firstStep + 1 + stepLength, 1);
            kneeSteps(:, j) = kneeTrajectory(i*firstStep + 1: i*firstStep + 1 + stepLength, 1);
            ankleSteps(:, j) = ankleTrajectory(i*firstStep + 1: i*firstStep + 1 + stepLength, 1);
            
            hipVelocitySteps(:, j) = hipVelocity(i*firstStep + 1: i*firstStep + 1 + stepLength, 1);
            kneeVelocitySteps(:, j) = kneeVelocity(i*firstStep + 1: i*firstStep + 1 + stepLength, 1);
            ankleVelocitySteps(:, j) = ankleVelocity(i*firstStep + 1: i*firstStep + 1 + stepLength, 1);
            
            interactionTorque(:, j) = loadTorque(i*firstStep + 1: i*firstStep + 1 + stepLength, 1);
            controlSignal(:, j) = desiredTorque(i*firstStep + 1: i*firstStep + 1 + stepLength, 1);
            
            kneeErrorSteps(:, j) = kneeError(i*firstStep + 1: i*firstStep + 1 + stepLength, 1);
            
            j = j + 1;
        end
    end
    
    Q_K_Mean(:,subject) = mean(Q_K)';
    Q_K_subject = [Q_K_subject; Q_K' subject*ones(size(Q_K'))];
    Q_K_STD(subject) = std(Q_K)
    u_two_norm_Mean(subject) = mean(u_two_norm);
    u_two_norm_STD(subject) = std(u_two_norm);
    u_two_norm_subjects = [u_two_norm_subjects; u_two_norm' subject*ones(size(u_two_norm'))];
    % Update total steps value to compute
    totalSteps = j - 1;
    
    % Computes the RMS value of each trajectory
    for i = 1 : totalSteps
        kneeStepsRMS(i) = rms(kneeSteps(:,i));
        
        % Computes the DC value of each step
        kneeStepsDC(i) = mean(kneeSteps(:,i));
        
        % Compute knee trajectory for each step without the DC component
        kneeStepsNoDC(:,i) = kneeSteps(:, i) - kneeStepsDC(i);
    end
    
    hipStepsMean(subject,:) = mean(hipSteps,2);
    kneeStepsMean(subject,:) = mean(kneeSteps,2);
    ankleStepsMean(subject,:) = mean(ankleSteps,2);
    
    hipVelocityStepsMean(subject,:) = mean(hipVelocitySteps,2);
    kneeVelocityStepsMean(subject,:) = mean(kneeVelocitySteps,2);
    ankleVelocityStepsMean(subject,:) = mean(ankleVelocitySteps,2);
    
    interactionTorqueMean(subject,:) = mean(interactionTorque,2);
    %interactionTorqueRMS(subject,:) = rms(interactionTorqueMean(subject,:),2);
    interactionTorqueSTD(subject,:) = std(interactionTorque,0,2);
    controlSignalMean(subject,:) = mean(controlSignal,2);
    
    kneeErrorStepsMean(subject,:) = mean(kneeErrorSteps,2);
    kneeErrorStepsSTD(subject,:) = std(kneeErrorSteps,0,2);
    clear hipSteps kneeSteps ankleSteps;
    clear hipVelocitySteps kneeVelocitySteps ankleVelocitySteps;
    clear interactionTorque controlSignal;
    clear kneeErrorSteps;
    clear Q_K u_two_norm;
    
end

%%
degreesOn = true;
sameFigure = false;

hipStepsDesired = testData(461:920,20,1);
kneeStepsDesired = testData(461:920,21,1);
ankleStepsDesired = testData(461:920,22,1) + pi/2;

if (degreesOn)
    hipStepsMean = rad2deg(hipStepsMean);
    kneeStepsMean = rad2deg(kneeStepsMean);
    ankleStepsMean = rad2deg(ankleStepsMean);
    
    hipVelocityStepsMean = rad2deg(hipVelocityStepsMean);
    kneeVelocityStepsMean = rad2deg(kneeVelocityStepsMean);
    ankleVelocityStepsMean = rad2deg(ankleVelocityStepsMean);
    
    interactionTorqueMean = interactionTorqueMean;
    controlSignalMean = controlSignalMean;
    
    hipStepsDesired = rad2deg(testData(461:920,20,1));
    kneeStepsDesired = rad2deg(testData(461:920,21,1));
    ankleStepsDesired = rad2deg(testData(461:920,22,1) + pi/2);
end

%% Hip trajectory
hipStepsMeanCorrected = [hipStepsMean(1:6,:); hipStepsMean(10,:)];

meanValue = mean(hipStepsMeanCorrected,1);
standardDeviation = std(hipStepsMeanCorrected);

if (~sameFigure)
    figure
end
plot(gait, meanValue, 'k', 'LineWidth',0.5);
hold on
h = plot(gait, hipStepsDesired, ':k');
set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
h = fill([gait fliplr(gait)],[(meanValue + standardDeviation) fliplr(meanValue - standardDeviation)], 'k', 'LineStyle', 'None');
alpha(alphaValue);
set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

%title('Hip joint')
ylabel('Position (deg)')
xlabel('Gait cycle (%)')
grid minor
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
fig = gcf;
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 paper_w paper_h];
saveas(gcf,'ImpedanceHip',figFormat)

%% Knee trajectory
meanValue = mean(kneeStepsMean,1);
standardDeviation = std(kneeStepsMean);

if (~sameFigure)
    figure
end
plot(gait, meanValue, 'k', 'LineWidth',0.5)
hold on
plot(gait, kneeStepsDesired, ':k');
h = fill([gait fliplr(gait)],[(meanValue + standardDeviation) fliplr(meanValue - standardDeviation)], 'k', 'LineStyle', 'None');
alpha(alphaValue)
set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
legend('Desired (Free)','Impedance','Location','Southwest')
%title('Knee joint')
ylabel('Position (deg)')
xlabel('Gait cycle (%)')
grid minor
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
fig = gcf;
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 paper_w paper_h];

saveas(gcf,'ImpedanceKnee',figFormat)

%% Ankle Trajectory
meanValue = mean(ankleStepsMean,1);
standardDeviation = std(ankleStepsMean);

if (~sameFigure)
    figure
end
plot(gait, meanValue, 'k','LineWidth',0.5)
hold on
h = plot(gait, ankleStepsDesired, ':k');
set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
h = fill([gait fliplr(gait)],[(meanValue + standardDeviation) fliplr(meanValue - standardDeviation)], 'k', 'LineStyle', 'None');
alpha(alphaValue)
set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

%title('Ankle joint')
ylabel('Position (deg)')
xlabel('Gait cycle (%)')
grid minor
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
fig = gcf;
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 paper_w paper_h];

saveas(gcf,'ImpedanceAnkle',figFormat)

if (sameFigure)
    legend('Hip','Knee','Ankle','Location','Best')
end

%% Error Hip trajectory
hipStepsMeanCorrected = [hipStepsMean(1:6,:); hipStepsMean(10,:)];

meanValue = mean(hipStepsMeanCorrected,1) - hipStepsDesired';
standardDeviation = std(hipStepsMeanCorrected);
errorRMS(1) = rms(meanValue);
errorMax(1) = max(meanValue);
errorMin(1) = min(meanValue);
peakDiff(1) = (max(abs(mean(hipStepsMeanCorrected,1)))-max(abs(hipStepsDesired')));
peakDiffPercent(1) = (max(abs(mean(hipStepsMeanCorrected,1)))-max(abs(hipStepsDesired')))/max(abs(hipStepsDesired'));
errorMaxPercent(1) = max(meanValue)/max(hipStepsDesired);
errorMinPercent(1) = min(meanValue)/min(hipStepsDesired);

if (~sameFigure)
    figure
end
plot(gait, meanValue, 'k', 'LineWidth',0.5);
hold on
%h = plot(gait, hipStepsDesired, ':k');
%set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
h = fill([gait fliplr(gait)],[(meanValue + standardDeviation) fliplr(meanValue - standardDeviation)], 'k', 'LineStyle', 'None');
alpha(alphaValue);
set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

%title('Hip joint')
ylabel('Position (deg)')
xlabel('Gait cycle (%)')
grid minor
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
fig = gcf;
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 paper_w paper_h];
saveas(gcf,'ImpedanceHipError',figFormat)

%% Knee trajectory Error
meanValue = -mean(kneeStepsMean,1) + kneeStepsDesired';
standardDeviation = std(kneeStepsMean);
errorRMS(2) = rms(meanValue);
errorMax(2) = max(meanValue);
errorMin(2) = min(meanValue);
peakDiff(2) = (max(abs(mean(kneeStepsMean,1)))-max(abs(kneeStepsDesired')));
peakDiffPercent(2) = peakDiff(2)./max(abs(kneeStepsDesired'));
errorMaxPercent(2) = max(meanValue)/max(kneeStepsDesired);
errorMinPercent(2) = min(meanValue)/min(kneeStepsDesired);
kneeRMS = rms(mean(kneeStepsMean,1));
desiredRMS = rms(kneeStepsDesired);
kneeRMSDifference = kneeRMS - desiredRMS;
kneeRMSDifferencePercent = kneeRMSDifference/desiredRMS;
if (~sameFigure)
    figure
end
figure(15)
plot(gait, meanValue, 'k', 'LineWidth',0.5)
hold on
%h = plot(gait, kneeStepsDesired, ':k');
%set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
h = fill([gait fliplr(gait)],[(meanValue + standardDeviation) fliplr(meanValue - standardDeviation)], 'k', 'LineStyle', 'None');
alpha(alphaValue)
%set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

%title('Knee joint')
ylabel('Position (deg)')
%xlabel('Gait cycle (%)')
grid minor
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
fig = gcf;
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 paper_w paper_h];

saveas(gcf,'ImpedanceKneeError',figFormat)

%% Ankle Trajectory
meanValue = mean(ankleStepsMean,1) - ankleStepsDesired';
standardDeviation = std(ankleStepsMean);
errorRMS(3) = sqrt(sum(meanValue.^2)/length(meanValue));
errorMax(3) = max(meanValue);
errorMin(3) = min(meanValue);
peakDiff(3) = (max(abs(mean(ankleStepsMean,1)))-max(abs(ankleStepsDesired')));
peakDiffPercent(3) = (max(abs(mean(ankleStepsMean,1)))-max(abs(ankleStepsDesired')))/max(abs(ankleStepsDesired'));
errorMaxPercent(3) = max(meanValue)/max(ankleStepsDesired);
errorMinPercent(3) = min(meanValue)/min(ankleStepsDesired);

if (~sameFigure)
    figure
end
plot(gait, meanValue, 'k','LineWidth',0.5)
hold on
%h = plot(gait, ankleStepsDesired, ':k');
%set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
h = fill([gait fliplr(gait)],[(meanValue + standardDeviation) fliplr(meanValue - standardDeviation)], 'k', 'LineStyle', 'None');
alpha(alphaValue)
%set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

%title('Ankle joint')
ylabel('Position (deg)')
xlabel('Gait cycle (%)')
grid minor
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
fig = gcf;
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 paper_w paper_h];

saveas(gcf,'ImpedanceAnkleError',figFormat)

if (sameFigure)
    legend('Hip','Knee','Ankle','Location','Best')
end
return

%% Hip velocity
meanValue = mean(hipVelocityStepsMean,1);
standardDeviation = std(hipVelocityStepsMean);

figure
plot(gait, meanValue, 'b')
hold on
fill([gait fliplr(gait)],[(meanValue + standardDeviation) fliplr(meanValue - standardDeviation)], 'b', 'LineStyle', 'None')
alpha(alphaValue)

%title('Hip joint')
ylabel('Velocity (deg/s)')
xlabel('Gait cycle (%)')
grid minor
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
fig = gcf;
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 paper_w paper_h];

saveas(gcf,'ImpedanceHipVel',figFormat)

%% Knee velocity
meanValue = mean(kneeVelocityStepsMean,1);
standardDeviation = std(kneeVelocityStepsMean);

figure
plot(gait, meanValue, 'r')
hold on
fill([gait fliplr(gait)],[(meanValue + standardDeviation) fliplr(meanValue - standardDeviation)], 'r', 'LineStyle', 'None')
alpha(alphaValue)

%title('Knee joint')
ylabel('Velocity (deg/s)')
%xlabel('Gait cycle (%)')
grid minor
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
fig = gcf;
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 paper_w paper_h];
saveas(gcf,'ImpedanceKneeVel',figFormat)

% Ankle velocity
meanValue = mean(ankleVelocityStepsMean,1);
standardDeviation = std(ankleVelocityStepsMean);

figure
plot(gait, meanValue, 'g')
hold on
fill([gait fliplr(gait)],[(meanValue + standardDeviation) fliplr(meanValue - standardDeviation)], 'g', 'LineStyle', 'None')
alpha(alphaValue)

%title('Ankle joint')
ylabel('Velocity (deg/s)')
xlabel('Gait cycle (%)')
grid minor
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
fig = gcf;
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 paper_w paper_h];
saveas(gcf,'ImpedanceAnkleVel',figFormat)

%% Interaction torque
meanValue = mean(interactionTorqueMean,1);
standardDeviation = std(interactionTorqueMean);

figure
plot(gait, meanValue, 'k', 'LineWidth',0.5)
hold on
fill([gait fliplr(gait)],[(meanValue + standardDeviation) fliplr(meanValue - standardDeviation)], 'k', 'LineStyle', 'None')
alpha(alphaValue)

%title('Knee joint')
ylabel('Interaction torque (Nm)')
xlabel('Gait cycle (%)')
grid minor
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
fig = gcf;
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 paper_w paper_h];

saveas(gcf,'ImpedanceInteractionTorque',figFormat)

%% Control signal
meanValue = mean(controlSignalMean,1);
standardDeviation = std(controlSignalMean);

figure
plot(gait, meanValue, 'k', 'LineWidth', 0.5)
hold on
fill([gait fliplr(gait)],[(meanValue + standardDeviation) fliplr(meanValue - standardDeviation)], 'k', 'LineStyle', 'None')
alpha(alphaValue)

%title('Control signal')
ylabel('Motor velocity (rpm)')
xlabel('Gait cycle (%)')
grid minor
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
fig = gcf;
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 paper_w paper_h];

saveas(gcf,'ImpedanceControlSignal',figFormat)

%% Interaction torque per subject
figure
for subject = 1 : subjectsTotal
    meanValue = interactionTorqueMean(subject,:);
    standardDeviation = interactionTorqueSTD(subject,:);
    
    subplot(5,2,subject)
    plot(gait, meanValue, 'r', 'LineWidth',0.5)
    hold on
    title(['RMS = ', num2str(rms(interactionTorqueMean(subject,:))),' Nm'])
    fill([gait fliplr(gait)],[(meanValue + standardDeviation) fliplr(meanValue - standardDeviation)], 'r', 'LineStyle', 'None')
    alpha(alphaValue)
    axis([0 1 -25 20])
    grid minor
    set(gca, 'FontName', 'CMU Serif')
end
%%
figure
for subject = 1 : 10
    meanValue = kneeErrorStepsMean(subject,:);
    standardDeviation = kneeErrorStepsSTD(subject,:);
    
    subplot(5,2,subject)
    plot(gait, meanValue, 'r', 'LineWidth',0.5)
    hold on
    title(['RMS = ', num2str(rms(interactionTorqueMean(subject,:))),' Nm'])
    fill([gait fliplr(gait)],[(meanValue + standardDeviation) fliplr(meanValue - standardDeviation)], 'r', 'LineStyle', 'None')
    alpha(alphaValue)
    
    grid minor
    set(gca, 'FontName', 'CMU Serif')
end
%%
%fprintf('Q_K = %f pm %f', mean(Q_K_Mean), std
%% Boxplots
figure
boxplot(Q_K_subject(:,1),Q_K_subject(:,2),'Symbol', '+k')
grid minor
set(gca, 'FontName', 'CMU Serif')
xlabel('Subject')
ylabel('Cumulative Controller Effort')
%%
figure
boxplot(u_two_norm_subjects(:,1)./mean(u_two_norm_subjects(:,1)),u_two_norm_subjects(:,2),'Symbol', '+k')
ylim([0 2])
grid minor
set(gca, 'FontName', 'CMU Serif')
xlabel('Subject')
ylabel('Normalized Controller Effort')
%Normalized by mean(u_two_norm_subjects(:,1))
%Mention the outliers that were suppressed
%%
figure
boxplot(interactionTorqueMean','Symbol', '+k')
grid minor
set(gca, 'FontName', 'CMU Serif')
xlabel('Subject')
ylabel('Interaction Torque (Nm)')