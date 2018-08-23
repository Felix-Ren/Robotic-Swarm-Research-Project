function output = gaussian(x,y,delta)
% gaussian  stamp pre-calculated blob function 
% param x, y    1x1 matrix    x y position of the robot
% returns       matrix        guassian distribution with same size of
% testbed
%         grid on the test bed and the values correspond to the third dimension.

    output = exp(-(x.^2+y.^2)/(2*delta^2))/(2*pi*delta^2);
end