% run row distribution simulation n times to find n statistics of number of
% steps to reach the steady state.

nLoop = 2;

% run main.m in a for loop and save the plots and critical steps
for i = 1:nLoop
    disp(i);
    main; 
end