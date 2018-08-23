function [nextX, nextY, currentState] = step_unif(robot, dimension, dt)
%STEP_UNIF Move robots according to control law and boundary law.
%   @param robot	struct		is a particular robot 
% 	@param dimension	1x2 matrix 	marks the boundary of the testbed
%	@param dt		scalar		is the step time

    k = 1.1; % should be a fixed number
    p = -0.1; % should be a integral of v3()
	lambda = 0; % global param relative to interruption boundary law
    speedCap = 1.32;

    % Switch the state of the robot according the probability
    if robot.state == 0
        currentState = rand() < k;
    else
        currentState = rand() > p;
    end
    
    if currentState == 0
        nextX = robot.x;
        nextY = robot.y;
    else 
        F = mapFunction_unif(robot.x, robot.y, dimension); % map is the scalar field
        D = 1/sqrt(F);  % let c=1
        dX = D*randn() *sqrt(dt);
        dY = D*randn() *sqrt(dt);
        
        dX = dX + robot.Fx;  % assume mass of a robot is 1?
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
  
		% check the new position of the robot and move it according to
		% boundary control laws if it is outside the boundaries
		[nextX, nextY] = ...
            bounceBack(robot.x, robot.y, nextX, nextY, ...
                        [0 dimension(1) 0 dimension(2)]);              
%             bounceBack_interruption(robot, nextX, nextY, ...
%                          [0 dimension(1) 0 dimension(2)], dt);            
%             bounceBack_specular(robot.x, robot.y, nextX, nextY, ...
%                         [0 dimension(1) 0 dimension(2)]);
%             bounceBack_rejection(robot.x, robot.y, nextX, nextY, ...
%                         [0 dimension(1) 0 dimension(2)]);            
%             bounceBack_multi_rejection(robot, nextX, nextY, ...
%                         [0 dimension(1) 0 dimension(2)], dt);                      
    end