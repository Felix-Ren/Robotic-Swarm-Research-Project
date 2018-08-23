% reset 
clc
clear
close all

% parameters 
filename = 'Data\N_delta_IC_2_3.mat';
nParam = 3;
gridSpacing = 200;
N = [5,10,50,200,400]; %[5,10,50,200,400,1000];
delta = [0.03, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.7]; %[0.1,0.3,0.5,0.7,0.9,1.1];
IC = [0.001,0.001;
      2.4,3.5;
      2.4,0;
      0,3.5];
 
aggregatedParams = zeros([length(N),length(delta),size(IC,1),nParam]);

%%
for i = 1:length(N)
    for j = 1:length(delta)
        fprintf('i: %f, j: %f\n',i,j)
        for k = 1:size(IC,1)
            try
                aggregatedParams(i,j,k,:) = main_unif(N(i),delta(j), ... 
                                                      IC(k,:),gridSpacing);
            catch
            end
        end
    end
end

%% save the result 
save(filename,'aggregatedParams')