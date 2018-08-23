function Z = mapFunction_unif(X, Y, dimension)
% fprintf('1'),X < 0
% fprintf('2'), X > dimension(1)
% fprintf('3'),(X < 0) | (X > dimension(1))
% fprintf('4'),Y < 0
% fprintf('5'),Y > dimension(2)
% fprintf('6'),(Y < 0) | (Y > dimension(2))
% fprintf('7'),((X < 0) | (X > dimension(1))) & ((Y < 0) | (Y > dimension(2)))
%if (X < 0 || X > dimension(1)) && (Y < 0 || Y > dimension(2))
%     if (0 <= X <= dimension(1) & 0 <= Y <= dimension(2))
%         Z = 1.0; %should be independent of the value on each pixel
%     else
%         Z = 0.0;
%     end
    Z = 1 * double(0 <= X <= dimension(1) & 0 <= Y <= dimension(2)) + 0; % z = 1 if the point is not in the one of the rows; z= 36 if it is.
end