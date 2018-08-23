% add noise to an exponential curve and use the exponential model
% to determine steady state. Compare the result with the true pre-
% noised exponential model

% reset
clc
clear ALL

% parameters
nLoop = 10^5;
sign2NoisRatio = 30;     % a positive number
limit = 10^12;           % consider expFit of this value is the asymptote
nStep = 980;             % approximately 3*steps needed to reach steady state
delta = 0.003;               % minor adjustment for 98% drop -> (98+delta*100)% drop
noiseParam = [0.496557790250829, 28.297556236044741, 0.621522346186021, ...
              0.492153150520979];  % noise from uniform distribution
noiseEVal = 0.623810826578511;     % expected value of the noise distribution
outputFile = ['Data\ExponentialWithNoise\delta', num2str(delta), '.mat'];
% expCounter = 1;
% expFitDelay = 180;       % start fitting the exponential model after these many steps

% Model: a*exp(b*x)+c
% % randomly generate constants from uniform distribution
% a = rand(1)*0.05+1.05;    % a <- [1.05, 1.1]
% b = rand(1)*0.001-0.015;  % b <- [-0.015, -0.014]
% c = rand(1)*0.01+0.66;    % c <- [0.66, 0.67]
a = 1.14;       % constanst from simulation with uniform distribution
b = -0.024;
c = 0.45;

x = 1:nStep;
fy = @(x) a*exp(b*x)+c;
y = fy(x);

% calculate 98%-drop line for the original exp curve
asymptoteExpOriginal = fy(limit);
expThresh = (fy(1)-asymptoteExpOriginal) * 0.02 + asymptoteExpOriginal;

%% find the critical steady state step according to the original exp curve
criticalStepOrigExp = find((y<expThresh), 1);

output = repmat(criticalStepOrigExp, [2, nLoop]);  
% critical step records. First row corresponds to original exp model; 
% second row corresponds to exp model with noise. To be output to file

for i = 1:nLoop
    % add noise
    noisePdf = @(t) 2*noiseParam(1)*noiseParam(2)*exp(-(noiseParam(2)^2)* ... 
                   (t-noiseParam(3)+noiseEVal).^2)/sqrt(pi);  % noise model from unif distribution
%     yWithNoise = awgn(y, sign2NoisRatio, 'measured');  % add noise
    noise = randpdf(noisePdf(-0.5:0.00001:0.5), -0.5:0.00001:0.5, size(y));
    yWithNoise = y + noise; 

    % detect steady state by applying exponential model
    % set the model: a*exp(b*x)+c
    expModelOpt = fitoptions('exp2');
    expModelOpt.lower = [-Inf -Inf -Inf 0];
    expModelOpt.upper = [Inf Inf Inf 0];
    expFit = fit(double(x)', yWithNoise', 'exp2', expModelOpt);

    % calculate (98+delta)%-drop line for exp model
    asymptoteFittedExp = expFit(limit);
    expModelThresh = (expFit(1)-asymptoteFittedExp) * (0.02-delta) ...
                     + asymptoteFittedExp;
 
    % find the critical steady state step according to the fitted exp model
    criticalStepFittedExp = find((expFit(x)<expModelThresh), 1);
    output(2,i) = criticalStepFittedExp;
    
    % save the results to file
    save(outputFile, 'output');
end

%% plot histogram of the two sets of critical steps
histogram(output(2,:)-output(1,:), 'Normalization', 'pdf');
ylabel('Density'); 
xlabel('Measured Steady State Step No. - Real Steady State Step No.'); 
title(['delta = ', num2str(delta*100), '%'])