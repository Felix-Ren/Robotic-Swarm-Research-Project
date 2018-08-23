function [ consts, steadyStateStart ] = expModelConst( nSteps, errorMetricVec )
%EXPMODELCONST Fit an exponential model (y = a*exp(b*t)+c) to given error metric values
%   @returns    consts              1x3 matrix  each column corresponds to a,b,c
%   @returns    steadyStateStart    scalar      the step number when state
%    state is reached

    % parameters
    limit = 10^12;            % consider expFit of this value is the asymptote
    expThreshPercent = 98.4;  % when the signal is below this number% drop from the max value,

    x = double(1:nSteps);

    % set the model: a*exp(b*x)+c
    expModelOpt = fitoptions('exp2');
    expModelOpt.lower = [-Inf -Inf -Inf 0];
    expModelOpt.upper = [Inf Inf Inf 0];
    expFit = fit(x', errorMetricVec, 'exp2', expModelOpt);
    
    % extract consts from the model
    consts = [expFit.a, expFit.b, expFit.c];

    % calculate (98+delta)%-drop line for exp model
    asymptote = expFit(limit);
    expModelThresh = (expFit(1)-asymptote) * (1-expThreshPercent/100) ... 
                     + asymptote;

    % find the critical steady state step according to the fitted exp model
    steadyStateStart = find((expFit(x)<expModelThresh), 1);

%     plotting        
%     figure();
%     fittedVal = expFit(x);
%     plot(x, errorMetricVec, 'blue', ...                 % error
%          x, fittedVal, 'red', ...                   % fitted value
%          x, ones(size(x,2))*expModelThresh, '--green')  % draw exp model threshold line
% 
%     axis tight
%     legend('error','fitted exponential curve',[num2str(expThreshPercent), ...
%            '% drop threshold'],'location','best')
%     ylabel('Error')
%     xlabel('Steps')
%     xlim([0 nSteps])
%     ylim([0 2]) 
end

