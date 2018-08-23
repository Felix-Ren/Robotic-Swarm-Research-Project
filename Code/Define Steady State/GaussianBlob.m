function [Z] = GaussianBlob( area, delta )
% GaussianBlob  Calculate a Gaussian blob on a rectangle of dimension
% 3 time width and 3 time length (601 x 601 grid) compard to the size of testbed (200 x 200 grid), centered at
% the center of the rectangle ((301,301)).
% param area    1x2 matrix    the dimension of the testbed
% returns       matrix        first and second dimension corresponding to 
%               3 time width and 3 time length test bed and the values correspond to the third dimension.

    binsPerAxis = 3*200;
    dx = 3*area(1)/binsPerAxis;
    dy = 3*area(2)/binsPerAxis;
    [X,Y] = meshgrid(linspace(0,3*area(1),binsPerAxis+1),linspace(0,3*area(2),binsPerAxis+1)); % X, Y are matrices
%     X = X(1:length(X)-1,1:length(X)-1);
%     Y = Y(1:length(Y)-1,1:length(Y)-1);
    Z = gaussian(X-1.5*area(1), Y-1.5*area(2), delta); 
end

