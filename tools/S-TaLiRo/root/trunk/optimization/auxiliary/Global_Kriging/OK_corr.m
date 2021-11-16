function R = OK_corr(corr_model,theta,D_X)
% calculate the correlation matrix for the modified nugget effect model
% corr_model - the correlation model used for the spatial correlation
% corr_model = 0: linear correlation function
% corr_model = 1: exponential correlation function
% corr_model = 2: gaussian correlation function
% corr_model = 3: cubic spline correlation function
% theta - the sensitivity parameters
% D_X - the distance matrix for the input locations
% Modified Nugget Effect Kriging toolbox. By YIN Jun, QUAN Ning, NG Szu
% Hui, 2011-2012.

d1 = size(D_X,1);
d2 = size(D_X,2);
d = size(theta,1);

switch corr_model
    case 0%linear correlation function
        R = prod(max(1 - abs(D_X) .* repmat(reshape(theta,[1 1 d]),[d1 d2]), 0));
    case 1%exponential correlation function
        R = exp(sum((-abs(D_X).^corr_model).*repmat(reshape(theta,[1 1 d]),[d1 d2]),3));
    case 2%Gaussian correlation function  
%         DX_squared = bsxfun(@power, abs(D_X), corr_model);
%         R= exp(sum(bsxfun(@times, -bsxfun(@power, abs(D_X), corr_model),reshape(theta,[1 1 d])),3));
        R = exp(sum(bsxfun(@times, -abs(D_X).^corr_model,reshape(theta,[1 1 d])),3));
%         R = exp(sum((-abs(D_X).^corr_model).*repmat(reshape(theta,[1 1 d]),[d1 d2]),3));
    case 3%Cubic correlation function
        T = repmat(reshape(theta,[1 1 d]),[d1 d2]);
        R = prod(((D_X<=(T./2)).*(1-6*(D_X./T).^2+6*(D_X./T).^3)+((T./2)<D_X & D_X<=T).*(2*(1-D_X./T).^3)),3);
end


end