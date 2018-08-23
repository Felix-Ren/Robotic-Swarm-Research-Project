% main routine
clear
close all
clc
% disp('new iteration')

dt = 0.5;                % step time
initX = 4;
initY = 6.06;
initState = 1;
nRobots = 200;
duration = 1200; %500;           % in seconds
robotRadius = 0.01;
dimension = [4.8, 7.0];  % x_max, y_max
CA = false;              % if use the experimental collision avoidance rule
prevTtlDist = 10^(-6);   % a random initial value
prevScore = 2;           % a random initial value
file1 = 'testCmultvDist_1_.txt';
file2 = 'testCmultvDist_2_.txt';
file4 = 'testCmultvDist_4_.txt';
file6 = 'testCmultvDist_6_.txt';
file8 = 'testCmultvDist_8_.txt';
file10 = 'testCmultvDist_10_.txt';
file12 = 'testCmultvDist_12_.txt';
file01 = 'Data\Envelope\rowLoopCriticalStep.txt';
peakSep = 31;
thresh = 1*10^(-4);
delay = 2*peakSep;         % delay of derivative calculation of the mean line
counter = 1;               % count how many steps output

robots = ...
    repmat(struct('x', initX, 'y', initY, 'state', 1, 'Fx', 0, 'Fy', 0),...
           nRobots, 1);
% each row == one robot, with [X, Y, State, Fx, Fy] % this is a structure
% array.
nSteps = int32(duration / dt);
score = zeros(nSteps, 1);
collisions = zeros(nSteps, 1);
avgCumulativTtlDist = zeros(1, nSteps);
avgCumulativScore = zeros(1, nSteps);
ttlDistVec = zeros(1, nSteps);

% % open the file to write info about cumulative total distance
% FID01 = fopen(file01, 'a+t');
% FID1 = fopen(file1, 'w');
% FID2 = fopen(file2, 'w');
% FID4 = fopen(file4, 'w');
% FID6 = fopen(file6, 'w');
% FID8 = fopen(file8, 'w');
% FID10 = fopen(file10, 'w');
% FID12 = fopen(file12, 'w');

for i=1:nSteps
    % ---- update x, y, state ----
    for j = 1:nRobots
        [robots(j).x, robots(j).y, robots(j).state] =...
            step(robots(j), dimension, dt); 
    end

    % ---- update repelling force ----
    if CA
        prevXs = [robots.x]; % copy constant values to enable parallel workers
        prevYs = [robots.y];
        parfor j=1:nRobots
            x = robots(j).x;
            y = robots(j).y;
            F_x = 0;
            F_y = 0;
            for k=1:nRobots
                [dF_x, dF_y] = repellingForce(prevXs(k) - x, ...
                    prevYs(k) - y, mapFunction(x, y, dimension));
                F_x = F_x + dF_x;
                F_y = F_y + dF_y;
            end
            robots(j).Fx = 0.5 * F_x * dt^2;
            robots(j).Fy = 0.5 * F_y * dt^2;
        end
    end
    
    %% Average cumulative distance
%     if i == 1
%         % check if i-1 is out of index bound
%         avgCumulativTtlDist(i) = sumAllDist(robots); 
%     else
%         avgCumulativTtlDist(i) = (avgCumulativTtlDist(i-1)*double((i-1)) + ...
%                                   sumAllDist(robots))/double(i); 
%         changeOfCmultvTtlDist = avgCumulativTtlDist(i)/avgCumulativTtlDist(i-1)-1;
%         
%         % write the step number to file when critical thresholds for change
%         % of cumulative total distances are reached.
%         if abs(changeOfCmultvTtlDist) < 10^(-12)
%             fwrite(FID12, ['At ', num2str(i), 'th step, the change is less than 10^(-12).    ']);
%         elseif abs(changeOfCmultvTtlDist) < 10^(-10)
%             fwrite(FID10, ['At ', num2str(i), 'th step, the change is less than 10^(-10).    ']);
%         elseif abs(changeOfCmultvTtlDist) < 10^(-8)
%             fwrite(FID8, ['At ', num2str(i), 'th step, the change is less than 10^(-8).    ']);
%         elseif abs(changeOfCmultvTtlDist) < 10^(-6)
%             fwrite(FID6, ['At ', num2str(i), 'th step, the change is less than 10^(-6).    ']);
%         elseif abs(changeOfCmultvTtlDist) < 10^(-4)
%             fwrite(FID4, ['At ', num2str(i), 'th step, the change is less than 10^(-4).    ']);
%         elseif abs(changeOfCmultvTtlDist) < 10^(-2)
%             fwrite(FID2, ['At ', num2str(i), 'th step, the change is less than 10^(-2).    ']);
%         elseif abs(changeOfCmultvTtlDist) < 10^(-1)
%             fwrite(FID1, ['At ', num2str(i), 'th step, the change is less than 10^(-1).    ']);
%         end
                             
%         fprintf('change of average of cumulative total distances: +/-%.12f percent.\n', ...
%                 abs(changeOfCmultvTtlDist)*100);
%     end
    % calculate sum of L2 distances between every pair of robots
%     totalDist = sumAllDist(robots); 
%     ttlDistVec(i) = totalDist;
%     disp(totalDist)           % print total distances to the console
%     fprintf('change of total distance: +/-%.6f percent.\n', ...
%             abs((totalDist - prevTtlDist)/prevTtlDist*100));
%     prevTtlDist = totalDist;  % update the previous total distance
    
%     % ---- plots, all in one figure ----
%     passives = robots([robots.state] == 0);
%     actives = robots([robots.state] == 1);
%     f1 = figure(1);
%     set(gca,'fontsize',18)
%     set(f1, 'Position', [50,50,600,600]);
%     if size(passives, 1) > 0 % 1 is the dimension
%         % subplot('position', [20 200 360 525]);
%         subplot('position', [0.06,0.4,0.37,0.55]);
%         plot([passives.x], [passives.y], 'bo'); % passive = blue
%         hold on
%     end
%     if size(actives, 1) > 0
%         subplot('position', [0.06,0.4,0.37,0.55]);
%         plot([actives.x], [actives.y], 'ro'); % active = red
%         hold on
%     end
% %     quiver([robots.x], [robots.y], [robots.Fx], [robots.Fy], 0);
%     hold off
%     axis([0 dimension(1) 0 dimension(2)])
%     title(['distribution @ time: ' , num2str(i * dt)]); % plot 1
%     % set(gca,'xtick',[]);
%     % set(gca,'ytick',[]);
%     
%     subplot('position', [0.48,0.4,0.45,0.55]);
    score(i) = coverageTest(robots, dimension, 0.2021); 
%     fprintf('change of error metric:   +/-%.6f percent.\n\n', ...
%             abs((score(i) - prevScore)/prevScore*100));
%     prevScore = score(i);  % update the previous score
    
    %% calculate cumulative error metric
%     avgCumulativScore(i) = mean(score(1:i));

    %% Define steady state
%     [envHighError, envLowError] = envelope(score(1:i), peakSep, 'peak');  % calculate envelope functions for existing errors
%     envMeanError = (envHighError+envLowError)/2;  % error curve after filter applied
%     if i > delay+1
%         meanLineDeriv_j = (envMeanError(i-delay) - envMeanError(i-delay-1))/dt;  % j = i - peakSep
%         if abs(meanLineDeriv_j) < thresh && counter <=5                          % consider steady state reached. Only write first 5 critical step numbers to file
% %             fprintf('%d step, derivative: %f\n', i-delay, meanLineDeriv_j);
% %             fprintf(FID01, ['At ', num2str(i-delay), ... 
% %                            'th step, the change is less than ', ...
% %                            num2str(thresh), '\n']);
%             fprintf(['At ', num2str(i-delay), ... 
%                            'th step, the change is less than ', ...
%                            num2str(thresh), '\n']);
%             counter = counter + 1;
%         end
%     end 
    
%     % Draw error, two envelope functions and mean function
%     f7 = figure(7);
%     plot(1:i,score(1:i), ...
%          1:i,envMeanError, ...
%          1:i,envHighError, ...
%          1:i,envLowError)
%     hold on
%     axis tight
%     legend('Error','Mean','location','best')
%     ylabel('Error')
%     xlabel('Steps')
%     ttl7 = ['\begin{tabular}{c}Row Distribution, Error v.s. Steps (Envelope) \\', ...
%           num2str(i), 'th step, delay = ', num2str(delay), ...
%           ', threshold = ', sprintf('%e', thresh), '\end{tabular}'];
%     title(ttl7,'interpreter','latex');
%     xlim([0 nSteps])
%     ylim([0 2]) 
% 
%     errorFrame(i) = getframe(f7);    % related to animation of plots of error and it envelope functions

    %% top view of blobs in this function (plot2)
%     title('gaussian coverage');
%     set(gca,'xtick',[]);
%     set(gca,'ytick',[]);
%     
%     % plot3
%     subplot('position', [0.06,0.06,0.8,0.3]);
%     plot(dt:dt:duration, score);
%     ylabel('error statistc');
%     xlabel('time (second)');
    
%     f3 = figure(3);
%     collisions(i) = detectCollision(robots, robotRadius);
%     plot(collisions);

%     robotFrame(i) = getframe(f1);  % related to animation of robots
end

% fclose(FID01);  % close the output file.
% fclose(FID1);  % close the output file.
% fclose(FID2);  % close the output file.
% fclose(FID4);  % close the output file.
% fclose(FID6);  % close the output file.
% fclose(FID8);  % close the output file.
% fclose(FID10);  % close the output file.
% fclose(FID12);  % close the output file.

% %% Draw total distance against iterations
% [envHighDist, envLowDist] = envelope(ttlDistVec, 50, 'peak');
% envMeanDist = (envHighDist+envLowDist)/2;
% 
% figure(4)
% f4 = plot(1:nSteps,ttlDistVec, ...
%      1:nSteps,envHighDist, ...
%      1:nSteps,envMeanDist, ...
%      1:nSteps,envLowDist);
%    
% axis tight
% legend('Total Distance','High','Mean','Low','location','best')
% ylabel('Total Distance of All Robots')
% xlabel('Steps')
% title('Total Distance v.s. Steps (Envelope, Analytic)')%% draw total distance against iterations

% %% Draw error metric against iterations at the end
% [envHighError, envLowError] = envelope(score, 31, 'peak');
% envMeanError = (envHighError+envLowError)/2;
% 
% figure(5)
% f5 = plot(1:nSteps,score, ...
%      1:nSteps,envHighError, ...
%      1:nSteps,envMeanError, ...
%      1:nSteps,envLowError);
%    
% axis tight
% legend('Error','High','Mean','Low','location','best')
% ylabel('Error')
% xlabel('Steps')
% title('Error v.s. Steps (Envelope)')
% 
%% Deviation of median line from true values at the end
% deviation = (envMeanError - score)/score;
% figure(6)
% f6 = plot(1:nSteps, deviation, ... 
%           1:nSteps, ones(1,nSteps).*(-0.05), ...
%           1:nSteps, ones(1,nSteps).*(0.05));
% title('Deviation of the Median Line from True Data')

%% draw error and cumulative error against steps
% f8 = figure(8);
% 
% subplot(2,1,1);
% plot(1:nSteps, score)
% ttl8_1 = ['\begin{tabular}{c}Row Distribution, Error v.s. Steps, ', ...
%       num2str(nSteps), ' steps\end{tabular}'];
% title(ttl8_1,'interpreter','latex');
% ylabel('Error Metric')
% xlim([0 nSteps])
% ylim([0 2]) 
% 
% subplot(2,1,2);
% plot(1:nSteps, avgCumulativScore)
% ttl8_2 = ['\begin{tabular}{c}Average of Cumulative Error Metric v.s. Steps\end{tabular}'];
% title(ttl8_2,'interpreter','latex');
% xlabel('Step')
% ylabel('Average Cumulative Error Metric')
% xlim([0 nSteps])
% ylim([0 2]) 

%% fit exponential decay to the error curve and draw meanLine fitted line
%  and error
xAxis = double(1:nSteps);
expFit = fit(transpose(xAxis), score, 'exp2');  % fit with one exponential term

f8 = figure(8);
% plot(1:nSteps, envMeanError, 'black')  % draw meanLine 
% hold on 
% 
plot(expFit, xAxis, score, 'cyan')
% hold off

axis tight
legend('Mean','Error','Fitted exponential curve','location','best')
ylabel('Error')
xlabel('Steps')
ttl7 = ['\begin{tabular}{c}Row Distribution, Error v.s. Steps (Envelope) \\', ...
      num2str(i), 'th step, delay = ', num2str(delay), ...
      ', threshold = ', sprintf('%e', thresh), '\end{tabular}'];
title(ttl7,'interpreter','latex');
xlim([0 nSteps])
ylim([0 2]) 

%% save the plot
timeOfCompletion = datestr(now,30);
saveas(f8, ['Plots\Envelope\double delay\error_row_31_10^-4_', ... 
            num2str(nSteps), 'steps', timeOfCompletion])
