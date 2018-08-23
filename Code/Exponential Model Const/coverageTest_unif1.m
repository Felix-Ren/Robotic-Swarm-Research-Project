function [statistic ] = coverageTest_unif1( robots, area, delta, gridSpacing )
%   @param gridSpacing is numberof grids per axis for the discretization

    binsPerAxis = gridSpacing;
    dx = area(1)/binsPerAxis;
    dy = area(2)/binsPerAxis;
    [X,Y] = meshgrid(linspace(-1*area(1),2*area(1),3*binsPerAxis+1),linspace(-1*area(2),2*area(2),3*binsPerAxis+1));
    X = X(1:length(X)-1,1:length(X)-1);
    Y = Y(1:length(Y)-1,1:length(Y)-1);
    
    Z_unwrapped = zeros(length(X), length(Y));  % size could be smaller if code is too slow
    
    for i = 1:size(robots,1)
%        Z = Z + gaussian(X-robots(i).x,Y-robots(i).y); 
%        Z = Z + gaussian(X-robots(i).x,Y-robots(i).y, delta); 
        Z_unwrapped = Z_unwrapped + exp(-((X-robots(i).x).^2+(Y-robots(i).y).^2)/(2*delta^2))/(2*pi*delta^2);
    end
    Z_unwrapped = Z_unwrapped / size(robots,1);
    
    % wrap around distributions outside the boundaries
    % divide the largger box to 9 smaller boxes with the testbed being one
    % of the smaller box
    Z = zeros(length(X)/3, length(Y)/3); 
    % summing up all 9 smaller boxes
    for i = 1:3
        for j = 1:3
            Z = Z + Z_unwrapped(1+binsPerAxis*(i-1):binsPerAxis*i, ... 
                                1+binsPerAxis*(j-1):binsPerAxis*j);
        end
    end
    functionGrid = mapFunction_unif(X((binsPerAxis+1):binsPerAxis*2, ... 
                                      (binsPerAxis+1):binsPerAxis*2), ...
                                    Y((binsPerAxis+1):binsPerAxis*2, ...
                                      (binsPerAxis+1):binsPerAxis*2),area);
    functionGrid = functionGrid / (area(1)*area(2));  % now each grid has value corresponding the pdf

%     % --- added plot here ---
%     S = surf(Z);
%     colormap('jet');
%     set(S, 'linestyle', 'none');
%     xlabel('x axis')
%     view(2);
%     h = colorbar;
    % set(h,'ytick',[]);
    Z = abs(Z - functionGrid);
    statistic = sum(sum(Z))*dx*dy;

%     function output = gaussian(x,y)
%         output = exp(-(x.^2+y.^2)/(2*delta^2))/(2*pi*delta^2);
%     end

    function output = circle(x,y)
        dist = x.^2+y.^2;
        output = dist;
        output(dist > delta^2) = 0;
        output(dist <= delta^2) = 1/(pi* delta^2);
    end
    
end

