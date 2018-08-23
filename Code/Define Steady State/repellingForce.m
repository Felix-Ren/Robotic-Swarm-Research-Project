function [F_x, F_y] = repellingForce(dx, dy, F)
    % take relative distance vector, and underlying field value
    % return Force in x and y direction
    % always dest relative to src
    C = 10; % controlling constant
    distance = sqrt(dx^2 + dy^2)+0.001; 
    if distance > 0.1
        F_x = 0;
        F_y = 0;
        return
    end
    ux = -1 * dx / distance;
    uy = -1 * dy / distance;
    magnitude = C / ( ((distance)^3 ) * F );
    F_x = magnitude * ux;
    F_y = magnitude * uy;
end