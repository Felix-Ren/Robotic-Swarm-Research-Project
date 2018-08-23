function [nextX, nextY] = bounceBack_multi_rejection(theRobot, X, Y, boundary, dt)
%BOUNCEBACK_MULTI_REJECTION use recursion to bounce the 
% robot back with multiple rejection boundary control law.
% (New increments are calculated until new position is in the box)
% Assume the start position of the robots is inside the boundaries
% @param theRobot is the robot we are concerned
% @param X,Y are the coordinates after the control law, may be outside and
% @param boundary box.
% @param dt is the time for each step
    
    if X >= boundary(1) && X <= boundary(2) ...
            && Y >= boundary(3) && Y <= boundary(4)
        nextX = X;
        nextY = Y;
    else
        [theRobot.x, theRobot.y, theRobot.state] =...
            step_unif(theRobot, [boundary(2) boundary(4)], dt); 
        nextX = theRobot.x;
        nextY = theRobot.y;
    end
end