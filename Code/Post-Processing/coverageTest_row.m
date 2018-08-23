function [statistic ] = coverageTest_row( xVec, yVec, area, delta )
%   @param  xVec  x-coordinates of all robot
%   @param  yVec  y-coordinates of all robot

    binsPerAxis = 200;
    N = length(xVec);   % number of robots
    dx = area(1)/binsPerAxis;
    dy = area(2)/binsPerAxis;
    [X,Y] = meshgrid(linspace(-1*area(1),2*area(1),3*binsPerAxis+1),linspace(-1*area(2),2*area(2),3*binsPerAxis+1));
    X = X(1:length(X)-1,1:length(X)-1);
    Y = Y(1:length(Y)-1,1:length(Y)-1);
    
    Z_unwrapped = zeros(length(X), length(Y));  % size could be smaller if code is too slow
    
    for i = 1:N
        Z_unwrapped = Z_unwrapped + exp(-((X-xVec(i)).^2+(Y-yVec(i)).^2)/(2*delta^2))/(2*pi*delta^2);
    end
    Z_unwrapped = Z_unwrapped / N;
    
    % wrap around distributions outside the boundaries
    % divide the largger box to 9 smaller boxes with the testbed being one
    % of the smaller box
    Z = zeros(length(X)/3, length(Y)/3);  % 200 x 200
    % summing up all 9 smaller boxes
    for i = 1:3
        for j = 1:3
            Z = Z + Z_unwrapped(1+200*(i-1):200*i,1+200*(j-1):200*j);
        end
    end
    
%     Z = Z_unwrapped(201:400,201:400) + Z_unwrapped(1:200,1:200) 
%         + Z_unwrapped(1:200,201:400) + Z_unwrapped(1:200,401:600)
%         + Z_unwrapped(201:400,1:200) + Z_unwrapped(201:400,401:600)
%         + Z_unwrapped(401:600,1:200) + Z_unwrapped(401:600,201:400)

    flowerWidth = 4.5/7;
    flowerLength = area(2);
    nFlowerRow = 3;
    rho1 = 1;
    rho2 = 36;
    ttlArea = area(1)*area(2);
    flowerArea = flowerWidth*flowerLength*nFlowerRow;
    scalarFieldNorm = rho1*(ttlArea-flowerArea)+rho2*flowerArea;
    functionGrid = mapFunction(X(201:400,201:400),Y(201:400,201:400),area);  % 200x200
    functionGrid = functionGrid / scalarFieldNorm;  % now each grid has value corresponding the pdf

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

