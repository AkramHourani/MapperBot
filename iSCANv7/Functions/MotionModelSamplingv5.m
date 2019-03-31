function [P_Pose, W_max]=MotionModelSamplingv5 (P_StartPose,Nbar,Bot)

% this function translates the pose randomly to a new pose based on the
% model explained in the paper

sigma = Bot.alpha * Nbar + Bot.Beta;
mu    = Nbar; % this is the mean number of steps
SIGMA = [sigma(1)^2, Bot.rho*prod(sigma); % Covariance matrix
    Bot.rho*prod(sigma),sigma(2)^2];

N = mvnrnd(mu,SIGMA,10); % the particles is multiplied by 10 in order to apply the thinning
W = mvnpdf(N,mu,SIGMA); % The wieght is the pdf value of the point

% Taking the samples with the highest wieght
[W_max,idx] = max(W);

P_Pose = OdoToCartV2( P_StartPose, N(idx,:),Bot);

end
