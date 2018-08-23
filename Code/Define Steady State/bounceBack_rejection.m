function [nextX, nextY] = bounceBack_rejection(prevX, prevY, X, Y, boundary)
%BOUNCEBACK_REJECTION Bounce the robot back with rejection 
% boundary control law. Assume the start position of the robots is inside the boundaries.
% @param prevX, prevY is the original coordiantes before this step
% @param X,Y are the coordinates after the control law, may out side and
% boundary box.

    if X >= boundary(1) && X <= boundary(2) ...
		&& Y >= boundary(3) && Y <= boundary(4)
        nextX = X;
        nextY = Y;
    else
        nextX = prevX;
        nextY = prevY;
    end
end