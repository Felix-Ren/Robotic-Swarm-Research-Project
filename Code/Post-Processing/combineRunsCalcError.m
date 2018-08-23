% reset 
clc
clear
close all

% parameters
filename1 = 'CSV Data/tracking';
filename3 = '.csv';
nFile = 200;
fileVec = [1:25,27:49,51:59,61:68,70:90,92:100,102:140,142:146,148:150, ...
           152:160,162:188,190:211];
nStep = 568;
testbedWidth = 121.92;  % cm
testbedLength = 177.8;
area = [4.8, 7.0];
delta = 0.2021;

%% clean data
xMatrix = zeros(nStep,nFile);  % Stores x-coordinates of all robots for all steps. 
                               % First dimension: step; second dimension: robots
yMatrix = zeros(nStep,nFile);
for i = 1:nFile  % go through all data files
    disp(i)
    robotCoord = extractCoord([filename1,num2str(fileVec(i)),filename3]);
    xMatrix(:,i) = robotCoord(:,1);
    yMatrix(:,i) = robotCoord(:,2);
end

%% convert x,y from cm to 10 inch scale
xMatrix = xMatrix.*area(1)./testbedWidth;
yMatrix = yMatrix.*area(2)./testbedLength;

%% compute error metric
errorMetric = zeros(nStep,1);
for i = 1:nStep  % calculate error metric for each step
    errorMetric(i) = coverageTest_row(xMatrix(i,:),yMatrix(i,:),area,delta);
end

%% check if steady state is reached. Plot error metric against step
disp('Steady state at:')
disp(fitExponential(errorMetric));