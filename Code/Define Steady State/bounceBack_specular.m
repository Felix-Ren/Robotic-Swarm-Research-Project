function [nextX, nextY] = bounceBack_specular(prevX, prevY, X, Y, boundary) 
%BOUNCEBACK_SPECULAR Use recursion to bounce the robot back with 
% specular boundary control law. Only get out of the function 
% when current position of the robot is within the boundaries.
% param     prevX, prevY is the original coordiantes before this step
% param     X,Y are the coordinates after the control law, may be outside the boundary box.
% param     boundary is a 1x4 matrix of doubles representing the left,
% 			right, top, and bottom boundary.
% returns   nextX, nextY which is the position after adjustment by the
% 			specular boundary control law.

    % check if the current position of robot is inside the box.
    if X >= boundary(1) && X <= boundary(2) ...
            && Y >= boundary(3) && Y <= boundary(4)
        nextX = X;
        nextY = Y;
        return
    end
    
    m = (Y - prevY) / (X - prevX);  % slope
    if X < boundary(1)  % robot is on the left side of the left boundary
        backX = X - boundary(1);
        X = boundary(1) - backX;
        % update prevX and prevY to be the point at which specular
        % reflection happens
        prevY = prevY + m*(boundary(1) - prevX);
        prevX = boundary(1);
    elseif X > boundary(2)  % robot is on the right side of the right boudnary 
        backX = X - boundary(2);
        X = boundary(2) - backX;
        prevY = prevY + m*(boundary(2) - prevX);
        prevX = boundary(2);        
    end
    
    m = (Y - prevY) / (X - prevX);
    if Y < boundary(3)  % robot is above the top boudnary 
        backY = Y - boundary(3);
        Y = boundary(3) - backY;
        prevX = prevX + (boundary(3) - prevY)/m;
        prevY = boundary(3); 
    elseif Y > boundary(4)  % robot is below the bottom boudnary 
        backY = Y - boundary(4);
        Y = boundary(4) - backY;
        prevX = prevX + (boundary(4) - prevY)/m;
        prevY = boundary(4); 
    end
    nextX = X;
	nextY = Y;
    
    % call bounceBack_specular recursively in case (X,Y) is still outside
    % the box.
%     [nextX, nextY] = bounceBack_specular(prevX, prevY, X, Y, boundary);
end