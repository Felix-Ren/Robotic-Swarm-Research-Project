% reset 
clc
clear 
close all

% params
dimension = [4.8, 7.0]; % x_max, y_max
d = sum(dimension.^2) ^.5;    % diameter of the domain
N = [5,10,50,200,400]; %[5,10,50,200,400,1000];
delta = [0.03, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.7]; %[0.1,0.3,0.5,0.7,0.9,1.1];
% non-dimensionalize delta
deltaOverDomain = delta./(d/2);
initialPos = 1; % 1-4, corresponding to run_main_unif IC


% load data
load('Data/N_delta_IC_2.mat')
% aggregatedParams: 1st dimension: number of robots; 2nd dimension: delta
% 3rd dimension: Initial position; 4th dimension: constants(a,b,c) 
dim1Size = size(aggregatedParams,1);
dim2Size = size(aggregatedParams,2);
dim3Size = size(aggregatedParams,3);
dim4Size = size(aggregatedParams,4);


% N,delta vs b
b_expected = 2/(d/pi);
bMatrixCell = cell(1,dim3Size);
% b is a positive constant
aggregatedParams(:,:,:,2) = aggregatedParams(:,:,:,2) .* (-1);

% create matrices of b values over all N and delta choices
for i = 1:dim3Size
    bMatrixCell{i} = aggregatedParams(:,:,i,2);
end

%% plotting
figure();
% surf(deltaOverDomain, N, bMatrixCell{1})
% colorbar

% plot b v.s. delta lines for all N's with first initial condition
for i = 1:dim1Size
    plot(deltaOverDomain, aggregatedParams(i,:,initialPos,2))
    hold on
end

axis tight
ttl = ['$b \textrm{ v.s. } \delta \textrm{ with different } N \textrm{, expected } b =', num2str(b_expected),'$'];
title(ttl, 'interpreter', 'latex')
% leg = {'error metric','fitted exponential curve',[num2str(expThreshPercent), ...
%        '\% drop threshold']};
% legend(leg, 'interpreter', 'latex', 'location','best')
% xlabel('$\frac{\delta}{r_\textrm{domain}}$','interpreter', 'latex')
% ylabel('$N (number of robots)$','interpreter','latex')
% zlabel('$b$','interpreter','latex')

xlabel('$\frac{\delta}{r_\textrm{domain}}$','interpreter', 'latex')
ylabel('$b$','interpreter','latex')
leg = {'N=5','N=10','N=50','N=200','N=400'};
legend(leg, 'interpreter', 'latex', 'location','best')
set(gca,'ticklabelinterpreter','latex', ...
        'Fontsize',20)