function [ output ] = stampGaussianBlob( Z,x, y, blob, gridSize_x, gridSize_y )
%STAMPGAUSSIANBLOB stamp the pre-calculated gaussian blob function on the
%given area
%   param  z                    the previous gaussian distribution on the
%   canvas
%   param  x,y                  position of the robot
%   param  blob  matrix         the precalculated blob function
%   stamped on (should be bigger than testbed size)
%   param  gridSize_x           grid size in x direction
%   param  gridSize_y           grid size in y direction
%   returns maxtrix             same size as area

    offsetX = (size(Z,1) - 200)/2;  %(800-200)/2
    offsetY = (size(Z,2) - 200)/2;  %(800-200)/2
    
    % first calculate the center of the blob function in terms of grid
    blobCenterX = ceil(size(blob, 1)/2);
    blobCenterY = ceil(size(blob, 2)/2);
    
    % locate robot on the canvas
    robotX = round(x/gridSize_x);
    robotY = round(y/gridSize_y);
    
    % adjust robotX, robotY in case they are outside testbed
    if robotX < 1
        robotX = 1;
    elseif robotX > 200
        robotX = 200;
    end
    if robotY < 1
        robotY = 1;
    elseif robotY > 200
        robotY = 200;
    end
    
    robotX = offsetX + robotX;
    robotY = offsetY + robotY;
    
    % locate top-left corner of the stamp
    tlcX = robotX-(blobCenterX-1);
    tlcY = robotY-(blobCenterY-1);
    
    Z((tlcX:(size(blob,1)-1+tlcX)),(tlcY:(size(blob,2)-1+tlcY))) =...
        Z((tlcX:(size(blob,1)-1+tlcX)),(tlcY:(size(blob,2)-1+tlcY)))+ blob;
    output = Z;
end

