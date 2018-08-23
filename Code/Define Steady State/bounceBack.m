function [nextX, nextY] = bounceBack(prevX, prevY, X, Y, boundary)
%BOUNCEBACK use recursion to bounce the robot back. Only get out of
% the function when current position of the robot is within the boundaries.

    if X > boundary(1) && X <= boundary(2) ...
            && Y > boundary(3) && Y <= boundary(4)
        nextX = X;
        nextY = Y;
        return
    end
    
    m = (Y - prevY) / (X - prevX);
    if X <= boundary(1)
        backX = X - boundary(1);
        X = boundary(1) - backX;
        Y = Y - 2*m*backX;
    elseif X > boundary(2)
        backX = X - boundary(2);
        X = boundary(2) - backX;
        Y = Y - 2*m*backX;
    end
    
    m = (Y - prevY) / (X - prevX);
    if Y <= boundary(3)
        backY = Y - boundary(3);
        Y = boundary(3) - backY;
        X = X - 2/m*backY;
    elseif Y > boundary(4)
        backY = Y - boundary(4);
        Y = boundary(4) - backY;
        X = X - 2/m*backY;
    end
    [nextX, nextY] = bounceBack(prevX, prevY, X, Y, boundary);
end