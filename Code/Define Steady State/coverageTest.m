function [statistic ] = coverageTest( robots, area, delta )

    binsPerAxis = 50; %200;
    dx = area(1)/binsPerAxis;
    dy = area(2)/binsPerAxis;
    [X,Y] = meshgrid(linspace(0,area(1),binsPerAxis+1),linspace(0,area(2),binsPerAxis+1));
    X = X(1:length(X)-1,1:length(X)-1);
    Y = Y(1:length(Y)-1,1:length(Y)-1);
    
    Z = zeros(length(X), length(Y));
    
    for i = 1:size(robots,1)
       Z = Z + gaussian(X-robots(i).x,Y-robots(i).y); 
    end
    Z = Z / size(robots,1);
    functionGrid = mapFunction(X,Y, area);  % mapFunction_unif(X,Y,area);
    functionGrid = functionGrid / (sum(sum(functionGrid))*dx*dy);
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

    function output = gaussian(x,y)
        output = exp(-(x.^2+y.^2)/(2*delta^2))/(2*pi*delta^2);
    end

    function output = circle(x,y)
        dist = x.^2+y.^2;
        output = dist;
        output(dist > delta^2) = 0;
        output(dist <= delta^2) = 1/(pi* delta^2);
    end
    
end

