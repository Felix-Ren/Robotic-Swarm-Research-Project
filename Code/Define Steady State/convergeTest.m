function [ score ] = convergeTest( robots, MAP )
% return true i the array of robots satisfies converge condition
    robotMap = zeros(size(MAP));
    for i=1:size(robots, 1)
        robotMap(ceil(robots(i).x), ceil(robots(i).y)) = ...
            robotMap(ceil(robots(i).x), ceil(robots(i).y)) + 1;
    end
    
    % normalize both maps
    normalMAP = MAP ./ sum(sum(MAP));
    normalRobotMap = robotMap ./ sum(sum(robotMap));
    misdisplacements = abs((normalRobotMap - normalMAP));% ./ normalMAP);
    score = sum(sum(misdisplacements)) / 2;
    % arbitrary thresholds: each block - 0.4; total satisfying blocks -0.3
end