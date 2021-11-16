function f = OK_Rlh_kd_nugget(params,k,d,D_X,Y,regr,corr_model, delta)
% Likelihood function for the parameter estimation of modified nugget effect model 
% params - parameters to be estimated, sigma_z and theta
% k,d - size of the input locations
% D_X - distance matrix for the input locations
% Y - observed simulation output values, size [k, 1], k points
% regr - the regression function
% sigma_e - the noise matrix
% corr_model - the correlation model used for the spatial correlation
% corr_model = 0: linear correlation function
% corr_model = 1: exponential correlation function
% corr_model = 2: gaussian correlation function
% corr_model = 3: cubic spline correlation function
% Modified Nugget Effect Kriging toolbox. By YIN Jun, QUAN Ning, NG Szu
% Hui, 2011-2012.

% if(min(params(1:d)) <= 0.001)
%     f = inf;
%     return;
% end

theta = params(1:d);
% get correlation matrix given theta
R = OK_corr(corr_model,theta,D_X);
% addition of nugget to increase stability of correlation matrix inversion
R = R + delta.*eye(size(R,1),size(R,2));   

% sum of determinisitc correlation matrix and noise matrix
CR  = R;
[U,pd] = chol(CR);
if(pd>0)
%     if pd1==0
%         U=U1;
%     elseif pd2==0
%         U=U2;
%     else
save data;
error('covariance matrix is nearly singular');
%     end
end

%
L = U';
Linv = L\eye(k);
Sinv = Linv'*Linv;

% the optimal beta given sigma_z and theta
beta = inv(regr'*Sinv*regr)*(regr'*(Sinv*Y)); 
sigma_z = (1/k)*(Y-regr*beta)'*Sinv*(Y-regr*beta);
%Z = L\(Y-regr*beta);

% negative log likelihood function
f = k*log(sigma_z)+log(det(R));
%f = (log(det(L)) + 0.5*Z'*Z + 0.5*k*log(2*pi));
% else
%     save data;
%     error('covariance matrix is nearly singular');
% end

if isnan(f) || isinf(f)
    f = k*log(sigma_z+eps(0))+log(det(R)+eps(0));
end

% Calculate the inverse matrix
end