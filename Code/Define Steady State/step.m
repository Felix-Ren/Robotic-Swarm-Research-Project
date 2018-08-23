% Move robots according to control law and boundary law.
function [nextX, nextY, currentState] = step(robot, dimension, dt)
    k = 1.1; % should be a fixed number
    p = -0.1; % should be a integral of v3()
    speedCap = 1.32;
    
    if robot.state == 0
        currentState = rand() < k;
    else
        currentState = rand() > p;
    end
    
    if currentState == 0
        nextX = robot.x;
        nextY = robot.y;
    else 
        F = mapFunction(robot.x, robot.y, dimension); % map is is the scalar field? What does it do?
        D = 1/sqrt(F); % let c=1
        dX = D*randn() *sqrt(dt);
        dY = D*randn() *sqrt(dt);
        
        dX = dX + robot.Fx;
        dY = dY + robot.Fy;
        
        % cap the (too) large steps
        len = (dX^2 + dY^2)^0.5;
        if len > speedCap * dt
            dX = dX * speedCap * dt / len;
            dY = dY * speedCap * dt / len;
        end
        
        % reflection <= No, just bounce back.
        nextX = robot.x + dX;
        nextY = robot.y + dY;
        
        [nextX, nextY] = ...
            bounceBack_conditional_rejection(robot.x, robot.y, nextX, ...
                                nextY, [0 dimension(1) 0 dimension(2)]);

%             bounceBack(robot.x, robot.y, nextX, nextY, ...
%                         [0 dimension(1) 0 dimension(2)]);
    end