function Z = mapFunction_ring(X, Y, dimension)
%MAPFUNCTION_RING Defines density of the ring target distribution
%   a point gets 36 if it is inside the ring and it gets 1 otherwise
%   @param X, Y    scalars     x,y coordinates of a robot
%   @dimension     1x2 matrix  width and length of the testbed
 
    Z = 1 + 35.*double((((X-dimension(1)/2).*(X-dimension(1)/2) ... 
        +(Y-dimension(2)/2).*(Y-dimension(2)/2))>=1.14*1.14)& ... 
        (((X-dimension(1)/2).*(X-dimension(1)/2)+(Y-dimension(2)/2) ...
        .*(Y-dimension(2)/2))<=2.06*2.06));
end