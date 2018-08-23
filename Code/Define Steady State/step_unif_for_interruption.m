% Move robots according to control law.
function [nextX, nextY, currentState] = step_unif_for_interruption(robot, dimension, dt)
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
        F = mapFunction_unif(robot.x, robot.y, dimension); % map is is the scalar field? What does it do?
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
    end