function [nextX, nextY] = bounceBack_conditional_rejection(prevX, prevY, X, Y, boundary)
%BOUNCEBACK_CONDITIONAL_REJECTION Bounce the robot back to its original 
% position if possible; or else bounce back as far as the distance of 
% the step allowed. Assume the start position of the robots is inside the boundaries.
% @param prevX, prevY is the original coordiantes before this step
% @param X,Y are the coordinates after the control law

    if X >= boundary(1) && X <= boundary(2) ...
		&& Y >= boundary(3) && Y <= boundary(4)
        nextX = X;
        nextY = Y;
    else
        
        % find the position of the robot when it hits a boundary
        m = (Y - prevY) / (X - prevX);
        if X < boundary(1)
            backX = X - boundary(1);
            bdryX = boundary(1);
            bdryY = Y - m*backX;
        elseif X > boundary(2)
            backX = X - boundary(2);
            bdryX = boundary(2);
            bdryY = Y - m*backX;
        end

        m = (Y - prevY) / (X - prevX);
        if Y < boundary(3)
            backY = Y - boundary(3);
            bdryY = boundary(3);
            bdryX = X - 1/m*backY;
        elseif Y > boundary(4)
            backY = Y - boundary(4);
            bdryY = boundary(4);
            bdryX = X - 1/m*backY;
        end
        
        % calculate the distances
        dist = sqrt((X-prevX)^2+(Y-prevY)^2);
        dist2Bdry = sqrt((bdryX-prevX)^2+(bdryY-prevY)^2);
        
        % check if the robot can return to its original position
        if dist2Bdry <= 0.5 * dist
            nextX = prevX;
            nextY = prevY;
        else
            [nextX, nextY] = bounceBack(prevX, prevY, X, Y, boundary);
        end
    end
end