% add noise to an exponential curve and use the exponential model
% to determine steady state. Compare the result with the true pre-
% noised exponential model

clc

% parameters
sign2NoisRatio = 30;     % a positive number
expFitDelay = 180;       % start fitting the exponential model after these many steps
limit = 10^12;           % consider expFit of this value is the asymptote
expCounter = 1;

% Model: a*exp(b*x)+c
% % randomly generate constants from uniform distribution
% a = rand(1)*0.05+1.05;    % a <- [1.05, 1.1]
% b = rand(1)*0.001-0.015;  % b <- [-0.015, -0.014]
% c = rand(1)*0.01+0.66;    % c <- [0.66, 0.67]
a = 1.08;
b = -0.015;
c = 0.66;

x = 1:800;
y = a*exp(b*x)+c;
fy = @(x) a*exp(b*x)+c;
%% add noise
% yWithNoise = awgn(x, sign2NoisRatio);
yWithNoise = awgn(y, sign2NoisRatio, 'measured');
% f1 = figure(1);
% plot(x', [y', yWithNoise'])
% legend('original error curve', 'error curve with noise', 'location', 'best')
% title(['Exponential Model: a = ', num2str(a), ',b = ', num2str(b), ...
%       ',c = ', num2str(c)])

%% detect steady state by applying exponential model
for i = x
    if i >= expFitDelay
        xAxis = double(1:i);
        
        % set the model: a*exp(b*x)+c
        expModelOpt = fitoptions('exp2');
        expModelOpt.lower = [-Inf -Inf -Inf 0];
        expModelOpt.upper = [Inf Inf Inf 0];
        expFit = fit(xAxis', yWithNoise(1:i)', 'exp2', expModelOpt);

        % calculate 2% line for exp model
        asymptote = expFit(limit);
        expModelThresh = (expFit(1)-asymptote) * 0.02 + asymptote;

        % calculate 2% line for the original exp curve
        asymptoteOriginal = fy(limit);
        expThresh = (fy(1)-asymptoteOriginal) * 0.02 + asymptoteOriginal;

%         % extract constants from the model
%         a = expFit.a;
%         b = expFit.b;
%         c = expFit.c;
%         d = expFit.d;      
        
        % compare the current value with the threshold
        if (expFit(double(i)) < expModelThresh) && expCounter <= 5 
           % steady state reached at this step 
           fprintf('At %dth step, steady state reached according to exp model.\n', i);
           expCounter = expCounter + 1;
        end   
    
        % plotting        
        f8 = figure(8);
        fittedVal = expFit(xAxis);
        plot(xAxis, y(1:i), 'blue', ...                  % original exponential
             xAxis, yWithNoise(1:i), 'cyan', ...         % exponential with noise
             xAxis, fittedVal, '-.red', ...                % fitted value
             xAxis, ones(1,i)*expThresh, '--green', ...  % threshold line of the original exp curve
             xAxis, ones(1,i)*expModelThresh, '-green')  % draw exp model threshold line

        
        axis tight
        legend('Original exponential curve','Exponential curve with noise', ...
               'Fitted exponential curve','Threshold of original exponential curve', ...
               'Threshold of fitted exponential curve','location','best')
        xlabel('Steps')
        xlim([0 size(x,2)])
        ylim([0 2]) 
        errorFrame(i) = getframe(f8);    % related to animation of plots of
                                         % error and it envelope functions
    end
end