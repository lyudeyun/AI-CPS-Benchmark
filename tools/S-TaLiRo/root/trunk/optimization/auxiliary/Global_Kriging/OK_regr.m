function regr = OK_regr(X,regr_model)
% Call the regression function for the MNEK model
% X - design locations for the simulation inputs, size [k, d], k points with d
% dimensions 
% regr_model - the underlying regression model for the mean function:
% regr_model = 0: constant mean function;
% regr_model = 1: linear mean function;
% regr_model = 2: quadratic mean function;
% Modified Nugget Effect Kriging toolbox. By YIN Jun, QUAN Ning, NG Szu
% Hui, 2011-2012.

%call regression function for the MNEK model
[length,dim] = size(X);
switch regr_model
    case 0
        regr = ones(length,1);
    case 1
        regr = [ones(length,1),X];
    case 2
        mid = (dim+1)*(dim+2)/2;
        regr = [ones(length,1),X,zeros(length,mid-dim-1)];
        j = dim+1;
        q = dim;
        for i = 1:dim
            regr(:,j+(1:q)) = repmat(X(:,i),1,q).*X(:,i:n)
        end
   
end
end