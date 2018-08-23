function [nextX, nextY] = bounceBack_interruption(theRobot, X, Y, boundary, dt)
%BOUNCEBACK_INTERRUPTION Assume robots move with constant speed, then each 
% step distance is same. Stops the robot when it hits the boundary and 
% calculate new step and move according to the new step for the rest of the time step.
% @param theRobot is contains x, y variables which are original position of the robot

    lambda = 0;
    
	% check if the robot is outside the boundaries
    while X < boundary(1) || X > boundary(2) || Y < boundary(3) || Y > boundary(4)
%     if X > boundary(1) && X <= boundary(2) ...
%             && Y > boundary(3) && Y <= boundary(4)
        
        % the new position is outside the boundary
        % calculate the position of the robot when it first hits a boundary
        [modX, modY] = boundRobot(theRobot.x, theRobot.y, X, Y, boundary);
        
        % calculate the proportion of time used in this time step
        deltaLambda = sqrt((modX - theRobot.x)^2 + (modY - theRobot.y)^2)/sqrt((X - theRobot.x)^2 + (Y - theRobot.y)^2);
        lambda = lambda + deltaLambda;
        
        % generate new step position
        theRobot.x = modX;
        theRobot.y = modY;
        [X,Y,theRobot.state] = step_unif_for_interruption(theRobot, [boundary(2) boundary(4)], dt);
        % X, Y are the new position after applying the step function
        
        % scale the distance from the wall to the new position by 1-lambda
        X = (1-lambda) * (X-modX) + modX;
        Y = (1-lambda) * (Y-modY) + modY;
    end

    % new position is inside boundary.
    nextX = X;
    nextY = Y;
end