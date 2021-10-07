%% ExoTaoHaptics Data Analysis
% Transparency Controller

%% Initialization
clear variables; close all; clc;
addpath('ExoData');

%% Plot configuration
alphaValue = 0.15;
fig_w = 16;
fig_h = 8;
paper_w = fig_w;
paper_h = fig_h;
figFormat = 'svg';

%% External data
subjectsTotal = 10;

testData(:,:,1) = load('data_22-02-2021_15-46-58.dat');
testData(:,:,2) = load('data_23-02-2021_17-07-27.dat');
testData(:,:,3) = load('data_24-02-2021_16-36-27.dat');
testData(:,:,4) = load('data_25-02-2021_15-53-03.dat');
testData(:,:,5) = load('data_26-02-2021_11-06-23.dat');
testData(:,:,6) = load('data_26-02-2021_20-37-48.dat');
testData(:,:,7) = load('data_04-03-2021_10-12-11.dat');
testData(:,:,8) = load('data_04-03-2021_16-08-57.dat');
testData(:,:,9) = load('data_04-03-2021_21-03-49.dat');
testData(:,:,10) = load('data_05-03-2021_11-11-49.dat');

%% Time data
time = testData(:,1,1);
dt = 5e-3;
timeLength = length(time);

%% Minimum peak heights to consider as heel strike
minPeakHeights = [-0.3, -0.2, 0.2, -0.5, -0.2, -0.2, -0.05, -0.2, 0, 0];
firstStep = 1e3*[3, 4, 2, 4, 6, 12, 2, 4, 4, 2.5];
lastStep = 1e3*[24, 24, 24, 24, 24, 24, 24, 17, 24, 24];

%% Peak value tolerance
peakTolerance = 0.3;

%% Step length
stepLength = 430;
minPeakDistance = 200;

% for subject = 1 : subjectsTotal
%     findpeaks(testData(:,3,subject), 'MinPeakHeight', minPeakHeights(subject),'MinPeakDistance', minPeakDistance)
%      input('')
% end

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
    
    gait = linspace(0,1,stepLength);
    
    % Find peaks (try to delimit each step)
    [peakValue, peakLocation, peakWidth, peakProminence] = findpeaks(kneeTrajectory, 'MinPeakHeight', 1.125*mean(kneeTrajectory), 'MinPeakDistance', minPeakDistance);
    % Compute the length of each step
    for i = 2 : length(peakLocation)
        peakLength(i) = peakLocation(i) - peakLocation(i-1) ;
    end
    
    % Compute average step length
    peakLengthMean = round(mean(peakLength));
    
    j = 1;
    for i = 1 : length(peakLocation) - 1
        % Check if it can start the step count
        if peakLocation(i) > firstStep(subject)
            if peakLocation(i) < lastStep(subject)
                currentStepLength(1,:) = peakLocation(i) : peakLocation(i + 1);
                
                T = (peakLocation(i + 1) - peakLocation(i))*dt + dt;
                dv = abs (max(desiredTorque(peakLocation(i) : peakLocation(i + 1))) - min(desiredTorque(peakLocation(i) : peakLocation(i + 1))) );
                Q_K(j) = 0;
                u_two_norm(j) = 0;
                
                for timeStep = peakLocation(i) : peakLocation(i + 1)
                    Q_K(j) = Q_K(j) + abs( desiredTorque(timeStep) - desiredTorque(timeStep-1) );
                    u_two_norm(j) = u_two_norm(j) + ( desiredTorque(timeStep)  * desiredTorque(timeStep)  + desiredTorque(timeStep - 1)  * desiredTorque(timeStep - 1)  ) * dt / 2;
                end
                Q_K(j) = Q_K(j) * (1 / T) * (1 / dv);

                hipSteps(:, j) = resample(hipTrajectory(peakLocation(i) : peakLocation(i + 1), 1), peakLengthMean, length(currentStepLength), 10);
                kneeSteps(:, j) = resample(kneeTrajectory(peakLocation(i) : peakLocation(i + 1), 1), peakLengthMean, length(currentStepLength), 10);
                ankleSteps(:, j) = resample(ankleTrajectory(peakLocation(i) : peakLocation(i + 1), 1), peakLengthMean, length(currentStepLength), 10);
                
                hipVelocitySteps(:, j) = resample(hipVelocity(peakLocation(i) : peakLocation(i + 1), 1), peakLengthMean, length(currentStepLength), 10);
                kneeVelocitySteps(:, j) = resample(kneeVelocity(peakLocation(i) : peakLocation(i + 1), 1), peakLengthMean, length(currentStepLength), 10);
                ankleVelocitySteps(:, j) = resample(ankleVelocity(peakLocation(i) : peakLocation(i + 1), 1), peakLengthMean, length(currentStepLength), 10);
                
                interactionTorque(:, j) = resample(loadTorque(peakLocation(i) : peakLocation(i + 1), 1), peakLengthMean, length(currentStepLength), 10);
                controlSignal(:, j) = resample(desiredTorque(peakLocation(i) : peakLocation(i + 1), 1), peakLengthMean, length(currentStepLength), 10);
                clear currentStepLength;
                j = j + 1;
            end
        end
    end
    Q_K_Mean(subject) = mean(Q_K);
    u_two_norm_Mean(subject) = mean(u_two_norm);
    Q_K_subject = [Q_K_subject; Q_K' subject*ones(size(Q_K'))];
    Q_K_STD(subject) = std(Q_K)
    u_two_norm_STD(subject) = std(u_two_norm);
    u_two_norm_subjects = [u_two_norm_subjects; u_two_norm' subject*ones(size(u_two_norm'))];
    
    stepsTotal(subject) = j - 1;
    
    kneeTrajectoryMean(subject) = mean(kneeTrajectory);
    kneeStepsLength(subject) = length(kneeSteps);
    
    kneeStepsMean(:, subject) = medfilt1(resample(mean(kneeSteps,2),stepLength,length(kneeSteps)),10,'truncate');
    kneeStepsSTD(:, subject) = std(mean(kneeSteps,2));
    
    controlSignalMean(:, subject) = medfilt1(resample(mean(controlSignal,2)', stepLength, length(kneeSteps)),10,'truncate');
    
    hipStepsMean(subject,:) = medfilt1(resample(mean(hipSteps,2), stepLength, length(kneeSteps)),10,'truncate');
    ankleStepsMean(subject,:) = medfilt1(resample(mean(ankleSteps,2), stepLength, length(kneeSteps)),10,'truncate');
    
    hipVelocityStepsMean(subject,:) = medfilt1(resample(mean(hipVelocitySteps,2), stepLength, length(kneeSteps)),10,'truncate');
    kneeVelocityStepsMean(subject,:) = medfilt1(resample(mean(kneeVelocitySteps,2), stepLength, length(kneeSteps)),10,'truncate');
    ankleVelocityStepsMean(subject,:) = medfilt1(resample(mean(ankleVelocitySteps,2), stepLength, length(kneeSteps)),10,'truncate');
    
    interactionTorqueMean(subject,:) = medfilt1(resample(mean(interactionTorque,2), stepLength, length(kneeSteps)),10,'truncate');
    
    clear kneeSteps hipSteps ankleSteps;
    clear hipVelocitySteps kneeVelocitySteps ankleVelocitySteps;
    clear interactionTorque controlSignal;
    clear Q_K u_two_norm;
   
end

%%
figure
ymin = min(min(kneeStepsMean));
ymax = max(max(kneeStepsMean));
for subject = 1 : subjectsTotal
    subplot(5,2,subject)
    plot(gait,(kneeStepsMean(:,subject)-mean(kneeStepsMean(:,subject))),'b')

    %ymin = min(kneeStepsMean(:,subject))
    %ymax = max(kneeStepsMean(:,subject))
    %axis([0 1 ymin ymax])
    %vline(0.6, 'k')
    text(0.7, 0.3, 'swing', 'FontName', 'CMU Serif')
    text(0.2, 0.3, 'stance','FontName', 'CMU Serif')
    set(gca, 'FontName', 'CMU Serif')
    
end

%%
figure
for subject = 1 : subjectsTotal
    plot(kneeStepsMean(:, subject))
    hold on
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
alphaValue = 0.15;
%% Hip trajectory
hipStepsMeanCorrected = [hipStepsMean(1:6,:)];

meanValue = mean(hipStepsMeanCorrected,1);
standardDeviation = std(hipStepsMeanCorrected);

figure
plot(gait, meanValue, 'k')
hold on
%plot(gait, hipStepsDesired, 'k')
fill([gait fliplr(gait)],[(meanValue + standardDeviation) fliplr(meanValue - standardDeviation)], 'k', 'LineStyle', 'None')
alpha(alphaValue)

%title('Hip joint')
ylabel('Position (deg)')
%xlabel('Gait percentage')
grid minor
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
fig = gcf;
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 paper_w paper_h/1.25];
saveas(gcf,'TransparencyHip','svg')

%% Knee trajectory
meanValue = mean(kneeStepsMean,2)';
standardDeviation = std(kneeStepsMean');

figure
plot(gait, meanValue, 'k')
hold on
%plot(gait, kneeStepsDesired, 'k')
fill([gait fliplr(gait)],[(meanValue + standardDeviation) fliplr(meanValue - standardDeviation)], 'k', 'LineStyle', 'None')
alpha(alphaValue)

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
saveas(gcf,'TransparencyKnee','svg')
%plot(gait,(180/pi)*resample([kneeStepsDesired(189:460); kneeStepsDesired(1:188)], 430, length(kneeStepsDesired)))
%hold on
%figure
%plot(gait,(180/pi)* resample(kneeStepsDesired,430, 460))
%hold on
%plot(gait,[meanValue(189:length(meanValue)) meanValue(1:188)])
%% Ankle Trajectory
meanValue = mean(ankleStepsMean,1);
standardDeviation = std(ankleStepsMean);

figure
plot(gait, meanValue, 'k')
hold on
%plot(gait, ankleStepsDesired, 'k')
fill([gait fliplr(gait)],[(meanValue + standardDeviation) fliplr(meanValue - standardDeviation)], 'k', 'LineStyle', 'None')
alpha(alphaValue)

%title('Ankle joint')
ylabel('Angle (deg)')
xlabel('Gait percentage')
grid minor
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
fig = gcf;
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 paper_w paper_h];
saveas(gcf,'TransparencyAnkle','svg')

%% Hip velocity
hipVelocityStepsMeanCorrected = [hipVelocityStepsMean(1:6,:)];

meanValue = mean(hipVelocityStepsMeanCorrected,1);
standardDeviation = std(hipVelocityStepsMeanCorrected);

figure
plot(gait, meanValue, 'b')
hold on
fill([gait fliplr(gait)],[(meanValue + standardDeviation) fliplr(meanValue - standardDeviation)], 'b', 'LineStyle', 'None')
alpha(alphaValue)

%title('Hip joint')
ylabel('Velocity (deg/s)')
%xlabel('Gait percentage')
grid minor
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
fig = gcf;
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 paper_w paper_h];
saveas(gcf,'TransparencyHipVel','svg')

% Knee velocity
meanValue = mean(kneeVelocityStepsMean,1);
standardDeviation = std(kneeVelocityStepsMean);

figure
plot(gait, meanValue, 'r')
hold on
fill([gait fliplr(gait)],[(meanValue + standardDeviation) fliplr(meanValue - standardDeviation)], 'r', 'LineStyle', 'None')
alpha(alphaValue)

%title('Knee joint')
ylabel('Velocity (deg/s)')
%xlabel('Gait percentage')
grid minor
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
fig = gcf;
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 paper_w paper_h];
saveas(gcf,'TransparencyKneeVel','svg')

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
xlabel('Gait percentage')
grid minor
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
fig = gcf;
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 paper_w paper_h];
saveas(gcf,'TransparencyAnkleVel','svg')

%% Interaction torque
meanValue = mean(interactionTorqueMean,1);
standardDeviation = std(interactionTorqueMean);

figure
plot(gait, meanValue, 'k')
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
fig.PaperPosition = [0 0 paper_w paper_h/1.25];
saveas(gcf,'TransparencyInteractionTorque','svg')
%%
% Control signal
meanValue = mean(controlSignalMean,2)';
standardDeviation = std(controlSignalMean');

figure
plot(gait, meanValue, 'k')
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
fig.PaperPosition = [0 0 paper_w paper_h/1.25];
saveas(gcf,'TransparencyControlSignal','svg')