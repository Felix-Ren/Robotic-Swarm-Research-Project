% clear;
% close all;
% clc;
format long;

%% uniform distribution error PDF calculation
% This code computes the CDF of the error for robots over a uniform
% distribution using monte carlo integration. The code then curve fits the
% CDF, then differentiates the curve fit expression to obtain a PDF of the
% error.

tic;
nRobots = 200;          % number of robots
nDim = 2*nRobots;       % number of vector dimensions
N = 100;                % number of evaluation points

delta = 0.2;            % robot radius
xmin = 0;               % left boundary
xmax = 4.8;             % right boundary
ymin = 0;               % bottom boundary
ymax = 7;               % top boundary
xmid = (xmin+xmax)/2;   % center, x
ymid = (ymin+ymax)/2;   % center, y
nx = 200;               % number of grid points, x
ny = nx;                % number of grid points, y
dx = (xmax-xmin)/nx;    % grid spacing, x
dy = (ymax-ymin)/ny;    % grid spacing, y
        
X = xmin:dx:xmax;       % x grid
Y = ymin:dy:ymax;       % y grid
[X,Y] = meshgrid(X,Y);  % grid
Xtens = repmat(X,1,1,N);    % tensor of X grids
Ytens = repmat(Y,1,1,N);    % tensor of Y grids

G_i = @(r_i) exp( - ( (Xtens-r_i(:,1,:)).^2 + (Ytens-r_i(:,2,:)).^2 ) / (2*delta^2) )...
    / (2*pi*delta^2);	% gaussian for each robot

rho_N = @(r) 0;
for i = 1:nRobots
    rho_N = @(r) rho_N(r) + G_i(r(:,2*i-1:2*i,:));
end
rho_N = @(r) rho_N(r)/nRobots;      % normalized density


% G = @(r) exp( - ( (Xtens - r).^2 + ().^2 ) / (2*delta^2) )...
%     / (2*pi*delta^2);


% monte carlo integration over uniform distribution...
rho = ones(size(X));    % uniform desired distribution
rho = rho/(sum(sum(rho))*dx*dy);    % normalization
rho = repmat(rho,1,1,N);            % tensor of desired distributions
Drho = @(r) abs(rho_N(r) - rho);    % distribution difference
e_N = @(r) sum(sum(Drho(r)))*dx*dy; % error metric

X0 = rand(1,nDim,N);        % N random robot configurations
X0(:,1:2:end,:) = (xmax-4*delta)*X0(:,1:2:end,:) + (xmin + 2*delta);    % shift and scale to testbed size
X0(:,2:2:end,:) = (ymax-4*delta)*X0(:,2:2:end,:) + (ymin + 2*delta);    % shift and scale to testbed size
%zz
e = e_N(X0);                % compute errors for robot configurations
eExpected = sum(e)/N;       % compute mean (expected) e value
% disp('uniform distribution expected error:');
% disp(eExpected);

% ee represents the error values over which the PDF is computed
nE = 10;                            % number of ee values
eemid = eExpected - mod(eExpected*100,5)/100;   % middle of ee domain
eemin = eemid - eemid/5;         	% min ee value
eemax = eemid + eemid/5;            % max ee value
dE = (eemax-eemin)/nE;              % ee grid space
eeX = linspace(eemin,eemax,nE);     % ee domain
eeX = repmat(eeX,1,1,N);            % ee domain for N configurations

% e = repmat(e,1,nE,1);
% rhoX = zeros(1,nE,N);           % PDF values
% rhoX(e <= eeX) = 1;             % configurations in set
% rhoX = permute(rhoX,[3,2,1]);   % move dimension three into rows
% I = sum(rhoX)/N;                % monte carlo mean
% eeX = eeX(:,:,1);               % restore ee domain
% rhoE = diff(I)/dE;              % differentiate CDF to get PDF
% eenew = linspace(eemin,eemax,nE-1); % new ee domain with one less point
% 
% timeElapsed = toc;
% 
% % curve fitting and plotting...
% fun = @(x,eeX) x(1)*erf(x(2)*(eeX-x(3)))+x(4);  % general erf form for CDF
% x0 = [0.5,20,0.77,0.5];                         % initial coefficient guess
% x = lsqcurvefit(fun,x0,eeX,I);                  % solve for coefficients
% disp('coefficients for error function curve fit:');
% disp(x);                                        % display coefficients
% nt = 1000;
% t = linspace(eeX(1),eeX(end),nt);            	% fine spaced ee domain
% figure();
% Ifit = fun(x,t);                                % CDF curve fit
% 
% plot(eeX,I,t,Ifit);         % plot computed and curve fit CDF
% xlim([eemin,eemax]);
% ttl = ['CDF for $N = $ ',num2str(nRobots),' Robots, Uniform Distribution'];
% title(ttl,'interpreter','latex');
% xlabel('Error Metric, $e_N^{\delta}$','interpreter','latex');
% ylabel('Cumulative Probability, $\mathcal{P}_e$','interpreter','latex');
% legend({'Numerical Approximation','Curve Fit'},'interpreter','latex',...
%     'location','southeast');
% set(gca,'ticklabelinterpreter','latex');
% timeOfCompletion = datestr(now,30);
% % name = ['uniformDistributionErrorPDF',timeOfCompletion,'.pdf'];
% % saveas(gca,name);
% % system(['pdfcrop ',name,' ',name]);
% 
% rhoFit = 2*x(1)*x(2)*exp(-(x(2)^2)*(t-x(3)).^2)/sqrt(pi);   % derivative of Ifit
% figure();
% plot(eenew,rhoE,t,rhoFit);  % plot computed and curve fit PDF
% xlim([eemin,eemax]);
% ttl = ['PDF for $N = $ ',num2str(nRobots),' Robots, Uniform Distribution'];
% title(ttl,'interpreter','latex');
% xlabel('Error Metric, $e_N^{\delta}$','interpreter','latex');
% ylabel('Probability Density, $\rho_e$','interpreter','latex');
% legend({'Numerical Approximation','Curve Fit'},'interpreter','latex',...
%     'location','northwest');
% set(gca,'ticklabelinterpreter','latex');
% % name = ['uniformDistributionErrorPDF',timeOfCompletion,'.pdf'];
% % saveas(gca,name);
% % system(['pdfcrop ',name,' ',name]);
% 
% % save(['uniformDistributionErrorPDF',timeOfCompletion,'.mat'],'eeX','I');