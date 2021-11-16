function [f,mse] = OK_Rpredict(model,X_pred,regr_model,Y)
% Build the modified nugget effect predictor based on the model given
% model - modified nugget effect kriging model, given by MNEK_model function 
% X_pred - locations to be predicted 
% regr_model - the underlying regression model for the mean function:
% regr_model = 0: constant mean function;
% regr_model = 1: linear mean function;
% regr_model = 2: quadratic mean function;

% Exmaple
%      M_predict  = MNEK_predict(model,X0,0);
% Using the parameter estimates of the MNEK model obtained from MNEK_model.m,
% the function predicts the response values at prediction points X0 with 
% a constant mean function

% Modified Nugget Effect Kriging toolbox. By YIN Jun, QUAN Ning, NG Szu
% Hui, 2011-2012.

% Obtain model parameters from MNEK_model
X = model.X;
min_X = model.min_X;
max_X = model.max_X;
[k,d] = size(X);
theta = model.theta;
beta = model.beta;
Rinv = model.Rinv;
sigma_z = model.sigma_z;
F = ones(k,1);

% get the size of the locations to be predicted
K = size(X_pred,1);
% get the regression model for the locations to be predicted
regr_pred = OK_regr(X_pred,regr_model);

% normalize distance matrix for prediction points and training points
X_pred = (X_pred - repmat(min_X,K,1)) ./ repmat(max_X-min_X,K,1);

distXpred = bsxfun(@minus, reshape(repmat(X, K, 1),[k K d]), reshape(X_pred,[1 K d]));

R_pred = OK_corr(2,theta,distXpred);

% calculate prediction responses and MSE at prediction points 
mse = NaN(K,1);
f = regr_pred*beta + R_pred'*(Rinv*(Y-ones(k,1)*beta));

FRFinv = 1/(F'*Rinv*F);
Rinv_Rpred = Rinv*R_pred;
for r = 1:K
    OneMinusFcrossR = 1-F'*Rinv_Rpred(:,r);
    mse(r,1) = sigma_z * (1 - R_pred(:,r)'*Rinv_Rpred(:,r) + (OneMinusFcrossR)'*FRFinv*(OneMinusFcrossR));
end
end
