%% reset
clc
clear

% parameters 
nLoop = 5;

steadyStateStartVec = [];

for i = 1:nLoop
   disp(i)
   main_row_exponential 
end