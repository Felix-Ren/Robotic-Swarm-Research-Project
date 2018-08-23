function [statistic ] = coverageTest_unif2( robots, area, delta, heatmapVec, blob)
% coverageTest_unif2 Use this function if used pre-calculated blobs
% param blob    (601x601) matrix    the pre-calculated gaussian blob functions
% returns       1x1 matrix        error between current distribution of robot and theoretical districution.

    maxGridOnOneSize = 800;  
    % 800 = 300 *2 (half of grid number on one size of pre-calculated gaussian
    % blob)+ 200 (grid number of original number of grid on one size of testbed)
    % for testbed
    binsPerAxis = 200;
    dx = area(1)/binsPerAxis;
    dy = area(2)/binsPerAxis;
    [X,Y] = meshgrid(linspace(0,area(1),binsPerAxis+1),linspace(0,area(2),binsPerAxis+1));
    X = X(1:length(X)-1,1:length(X)-1);
    Y = Y(1:length(Y)-1,1:length(Y)-1);
    Z = zeros(maxGridOnOneSize, maxGridOnOneSize); % create a matrix of 0's of the given size
    
    % sum to get the gussian blob density
    for i = 1:size(robots,1)
       Z = stampGaussianBlob(Z, robots(i).x, robots(i).y, blob, dx, dy); 
    end
    Z = Z / size(robots,1);
    functionGrid = mapFunction_unif(X,Y, area);
    functionGrid = functionGrid / (sum(sum(functionGrid))*dx*dy);
    assignin('caller',inputname(4),heatmapVec+Z)
    % --- added plot here ---
    S = surf(Z(301:500,301:500));  % only draw heat map with the testbed area
%     S = surf(Z);  % only draw heat map with the testbed area
    colormap('jet');
    set(S, 'linestyle', 'none');
    xlabel('x axis')
    view(2);
    h = colorbar;
    % set(h,'ytick',[]);
    Z = abs(Z(301:500,301:500) - functionGrid);
    statistic = sum(sum(Z))*dx*dy;  % this statistic is not accurate because 
    % it has not been adjusted by wrapping around distributions

    function output = circle(x,y)
        dist = x.^2+y.^2;
        output = dist;
        output(dist > delta^2) = 0;
        output(dist <= delta^2) = 1/(pi* delta^2);
    end
    
end

