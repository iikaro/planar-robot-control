%% Subjects Data
subjectIndex = 1:6;   
age = [];       %years
height = [];    %cm
mass = [];      %
sex = [];       %M/F

%% Statistics
ageMean = mean(age);
ageDeviation = std(age);

heightMean = mean(height);
heighDeviation = std(height);

massMean = mean(mass);
masDeviation = std(mass);

femaleCount = 0;
maleCount = 0;
for i = 1 : length(subjectIndex)
    if sex(i) == 'F'
        femaleCount = femaleCount + 1;
    else
        maleCount = maleCount + 1;
    end
end

%% 
