% main routine using uniform scalar field
clear
close all
clc

dt = 0.5; % step time (second/step)
initX = 4;
initY = 6.06;
initState = 1;
nRobots = 200;
duration = 600; % 300;  % seconds
robotRadius = 0.01;
dimension = [4.8, 7.0]; % x_max, y_max
CA = false; % whether use the experimental collision avoidance rule
delta = 0.2021;  % parameter used to calculate gaussian blob
peakSep = 31;
thresh = 1*10^(-4);
delay = 2*peakSep;         % delay of derivative calculation of the mean line

% this following block of code is related to cumulative heatmaps
heatmapData = zeros(200,200); % store each heatmap data at desired time point
garbage = zeros(200,200); 
heatmapStart = 1; % the step where heatmap starts being collected and summed
% blob = GaussianBlob(dimension, delta);

robots = ...
    repmat(struct('x', initX, 'y', initY, 'state', 1, 'Fx', 0, 'Fy', 0),...
           nRobots, 1);
% each row == one robot, with [X, Y, State, Fx, Fy]. Fx, Fy are forces used 
% in the collision cases
% this is a structure array.
nSteps = int32(duration / dt);
score = zeros(nSteps, 1);
collisions = zeros(nSteps, 1);

for i=1:nSteps
    % ---- update x, y, state ----
    for j = 1:nRobots
        [robots(j).x, robots(j).y, robots(j).state] =...
            step_unif(robots(j), dimension, dt); 
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
                    prevYs(k) - y, mapFunction_unif(x, y, dimension)); % bug here. x here is an array ( 200 * 200)
                F_x = F_x + dF_x;
                F_y = F_y + dF_y;
            end
            robots(j).Fx = 0.5 * F_x * dt^2;
            robots(j).Fy = 0.5 * F_y * dt^2;
        end
    end
    
%     % ---- plots, all in one figure ----
% %     figure
% %     HMobj = HeatMap(robots.x, robots.y)
%     passives = robots([robots.state] == 0);
%     actives = robots([robots.state] == 1);
%     f1 = figure(1);
%     set(gca,'fontsize',18)
%     set(f1, 'Position', [50,50,600,600]);
% 	% plot robots of the two states seperately with two different colors
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
% 	subplot('position', [0.48,0.4,0.45,0.55]);
    if i >= heatmapStart  
	% if the step reached the start of the step for collecting cumulative heatmap,
	% then overlap the heatmaps
%         score(i) = coverageTest_unif2(robots, dimension, 0.2021, heatmapData, blob); % also plot the 
        % top view of blobs in this function (plot2)
        
        % draw the bottom plot and calculate the error for each time step
        score(i) = coverageTest_unif1(robots, dimension, 0.2021, heatmapData);
        
    else % feed the function with empty vector as last input to turn off heatmap collection when
        % time step has not reached heatmapStart
%         score(i) = coverageTest_unif2(robots, dimension, 0.2021, garbage, blob);

        % draw the bottom plot and calculate the error for each time step
        score(i) = coverageTest_unif1(robots, dimension, 0.2021, garbage);
    
    end

    %% Define steady state
    [envHighError, envLowError] = envelope(score(1:i), peakSep, 'peak');             % calculate envelope functions for existing errors
    envMeanError = (envHighError+envLowError)/2;                                     % error curve after filter applied
    if i > delay+1
        meanLineDeriv_j = (envMeanError(i-delay) - envMeanError(i-delay-1))/dt;  % j = i - peakSep
        if abs(meanLineDeriv_j) < thresh                                          % consider steady state reached
            fprintf('%d step, derivative: %f\n', i-delay, meanLineDeriv_j);
        end
    end 
    
    % Draw error, two envelope functions and mean function
    f7 = figure(7);
    plot(1:i,score(1:i), ...
         1:i,envHighError, ...
         1:i,envMeanError, ...
         1:i,envLowError)
    axis tight
    legend('Error','High','Mean','Low','location','best')
    ylabel('Error')
    xlabel('Steps')
    ttl = ['\begin{tabular}{c}Uniform Distribution, Error v.s. Steps (Envelope) \\', ...
          num2str(i), 'th step, delay = ', num2str(delay), ...
          ', threshold = ', sprintf('%e', thresh), '\end{tabular}'];
    title(ttl,'interpreter','latex');
    xlim([0 nSteps])
    ylim([0 2]) 

%     title('gaussian coverage');
%     set(gca,'xtick',[]);
%     set(gca,'ytick',[]);
%     
%     %% plot3
%     subplot('position', [0.06,0.06,0.8,0.3]);
%     plot(dt:dt:duration, score);
%     ylabel('error statistc');
%     xlabel('time (second)');
%     %set(gca, 'ytick', [])
%     %set(gca, 'ytick', [0,1.0*10^(4),2.0*10^(4),3.0*10^(4),4.0*10^(4),5.0*10^(4)])
%     
% %     f3 = figure(3);
% %     collisions(i) = detectCollision(robots, robotRadius);
% %     plot(collisions);
% 
%     robotFrame(i) = getframe(f1); % related to animation of robots
    errorFrame(i) = getframe(f7);    % related to animation of plots of error and it envelope functions
end

% % normalize the accumulated heatmap
% stdHeatmapData = heatmapData ./(double(nSteps) - heatmapStart + 1);
% % plot the overlapped heatmap
% f2 = figure(2)
% set(f2,'name','specular law start at 340th step','numbertitle','off')
% S = surf(stdHeatmapData);
% colormap('jet');
% set(S, 'linestyle', 'none');
% view(2);
% h = colorbar;
    
% % print stats of error metrics after the system reach the equilibrium
% % states (last 60 sec)
% scoreAfterEq = sort( score((end - 60/dt + 1):end));
% fprintf('mean: %d\n', mean(scoreAfterEq))
% fprintf('25%s: %d\n', '%', scoreAfterEq(uint8(size(scoreAfterEq,1)/4))) 
% fprintf('50%s: %d\n', '%', median(scoreAfterEq)) 
% fprintf('75%s: %d\n', '%', scoreAfterEq(uint8(size(scoreAfterEq,1)/4*3))) 