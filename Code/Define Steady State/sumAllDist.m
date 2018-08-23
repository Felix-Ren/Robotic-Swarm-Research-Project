function [ ttlDist ] = sumAllDist( robots )
%SUMALLDIST Calculate summation of distance between every pair of robots
%   This indicator is only meaningful for a given system, i.e.
%   given number of robots, target distribution, grid spacing, and testbed
%   dimension
%   @param robots  vector of struct  has x,y coordinates for each robot
%   @returns       the total L2 distance
    
    N = size(robots, 1);  % the size of the vector
    ttlDist = 0;          % initialize the return value
    
    % go through all pairs of robots 
    for i = 1:(N-1)
       for j = (i+1):N
           ttlDist = ttlDist + norm([robots(i).x; robots(i).y] - ...
                            [robots(j).x; robots(j).y], 2);
       end        
    end
end

