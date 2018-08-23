function [ const ] = main_unif(N,delta,initPos,gridPerAxis)
%   @param  N       number of robots
%   @param  delta   robot effective task region radius (related to gaussian
%                   blob calculations
%   @param  initPos  1x2 matric  initial position of all robots

    dt = 0.5;               % step time (second/step)
    initX = initPos(1);
    initY = initPos(2);
    nRobots = N;
    duration = 800/2;         % seconds
    robotRadius = 0.01;
    dimension = [4.8, 7.0]; % x_max, y_max
%     delta = 0.2021;         % parameter used to calculate gaussian blob

    % blob = GaussianBlob(dimension, delta);

    robots = ...
        repmat(struct('x', initX, 'y', initY, 'state', 1, 'Fx', 0, 'Fy', 0),...
               nRobots, 1);
    % each row == one robot, with [X, Y, State, Fx, Fy]. Fx, Fy are forces used 
    % in the collision cases
    % this is a structure array.
    nSteps = int32(duration / dt);
    score = zeros(nSteps, 1);

    for i=1:nSteps
        % ---- update x, y, state ----
        for j = 1:nRobots
            [robots(j).x, robots(j).y, robots(j).state] =...
                step_unif(robots(j), dimension, dt); 
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
    %         score(i) = coverageTest_unif2(robots, dimension, 0.2021, heatmapData, blob); % also plot the 
            % top view of blobs in this function (plot2)

            % draw the bottom plot and calculate the error for each time step
        score(i) = coverageTest_unif1(robots, dimension, delta, gridPerAxis);

    %     title('gaussian coverage');
    %     set(gca,'xtick',[]);
    %     set(gca,'ytick',[]);
    %     
    %     % plot3
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
    end

    [const, steadyState] = expModelConst(nSteps, score);
    disp('Steady state reached at ')
    disp(steadyState)

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
end