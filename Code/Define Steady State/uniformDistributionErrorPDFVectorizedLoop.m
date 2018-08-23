clc
close all
clear

nLoop = 30;

% initialize containers
% sigmaVec = zeros(1, nLoop);
% muVec = zeros(1, nLoop);
sigmaVec = [];
muVec = [];

% collect nLoop mu's and sigma's
for i = 1:nLoop
    disp(i)
    uniformDistributionErrorPDFVectorized
%     sigmaVec(i) = std(e);
%     muVec(i) = eExpected;
    sigmaVec = [sigmaVec std(e)];
    muVec = [muVec eExpected];

    % reset e and eExpected
    clear e eExpected
end