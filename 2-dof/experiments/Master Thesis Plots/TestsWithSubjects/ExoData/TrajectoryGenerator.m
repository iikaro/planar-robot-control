%%  ExoTaoHaptics - Haptic Interface for Lower Limbs Exoskeleton
%   Author: Icaro Ostan
%   Date: 30/11/2020
%% Trajectory Generator

clear variables; %close all; clc;
fileName = ['data_09-02-2021_16-41-38.dat'; 'data_09-02-2021_16-44-39.dat';...
    'data_09-02-2021_16-59-47.dat'; 'data_09-02-2021_17-02-35.dat'];
%datalog = load('data_09-02-2021_16-41-38.dat');   %1km/h
%datalog = load('data_09-02-2021_16-44-39.dat');   %2km/h
%datalog = load('data_09-02-2021_16-59-47.dat');   %1km/h
%datalog = load('data_09-02-2021_17-02-35.dat');   %2km/h

%data = load('data_10-02-2021_17-12-29.dat');
%data = load('data_10-02-2021_17-36-16.dat');
ex
for dataNumber = 1 : 4
    data = load(fileName(dataNumber,:));
    %% External data
    time = data(:,1);
    angle = [data(:,2), data(:,3), data(:,4)];
    qDesired = [data(:,20), data(:,21), data(:,22)];
    %% Time
    dt = 5e-3;
    timeLength = length(time);
    
    %% Angles
    % figure
    % plot(time, angle(:,2), time, qDesired(:,2))
    % title('Knee Joint Displacement (rad)')
    
    %% Identify peaks and valleys
    knee = angle(:,2);
    [peaks,locs,w,p] = findpeaks(knee);
    
    %% Step lengths calculation
    j = 1;
    for i = 2 : length(locs)
        if (locs(i) - locs(i-1) > 400)
            %if (locs(i) - locs(i-1) < 600 & locs(i) - locs(i-1) > 400)
            stepLength(j) = locs(i) - locs(i-1);
            j = j + 1;
        end
    end
    
    %% Average step length calculation
    j = 1;
    p = round(mean(stepLength));
    p = 460;
    % Ignore first 4 peaks (first steps)
    locs = locs(5:length(locs));
    
    %figure
    for i = 1 : length(locs) - 1
        trajectoryPerStep = knee(locs(i) : locs(i+1));
        
        meanTrajectoryValue(j) = mean(trajectoryPerStep);
        %pause(0.5)
        %hold on
        if abs(meanTrajectoryValue(j)) < 0.5*abs(mean(meanTrajectoryValue))
            clear trajectoryPerStep
            i
        else
            %plot(trajectoryPerStep);
            trajectoryPerStepResampled(:, j) = resample(trajectoryPerStep, p, length(trajectoryPerStep));
            j = j + 1;
            clear trajectoryPerStep
        end
    end
    
    trajectoryPerStepResampledFree = trajectoryPerStepResampled(:,1:18);
    trajectoryPerStepResampledHolding = trajectoryPerStepResampled(:,19:size(trajectoryPerStepResampled,2));
    
    %figure
    %plot(trajectoryPerStepResampledFree)
    %figure
    %plot(trajectoryPerStepResampledHolding)
    
    %% Resampled trajectories
    % figure
    % plot(mean(trajectoryPerStepResampledFree'))
    % hold on
    % plot(mean(trajectoryPerStepResampledHolding'))
    
    %% Final average trajectory
    meanKneeTrajectory(:,dataNumber) = mean(trajectoryPerStepResampled')';
    plot(1:length(trajectoryPerStepResampledFree), repmat(meanKneeTrajectory,1)*180/pi)
    hold on
end
%%
averageMeanKneeTrajectory = mean(meanKneeTrajectory')
figure
plot(averageMeanKneeTrajectory)
%% Export to .dat file
save('kneeSetpoints.dat', 'meanKneeTrajectory', '-ASCII');