function nCollisions = detectCollision(robots, radius)
    nCollisions = 0;
    nRobots = size(robots, 1);
    for i = 1:nRobots
        cx = robots(i).x;
        cy = robots(i).y;
        n = 0; % number of colliding robots with current one
        for j = 1:nRobots
            d = sqrt((robots(j).x - cx)^2 + (robots(j).y - cy)^2);
            if d < radius
                n = n + 1;
            end
        end
        if n > 1 % excluding collision with itself
            nCollisions = nCollisions + 1;
        end
    end
end