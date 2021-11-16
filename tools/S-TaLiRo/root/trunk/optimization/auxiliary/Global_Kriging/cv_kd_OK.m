function f = cv_kd_OK(x, y, alpha_quatile)
%alpha_quatile corresponds to the number that yeidls the desired quantile
%of a standard normal distribution i.e. 95% ==> 1.97
    n = size(x,1);
    f = zeros(n,1);
 
    for i = 1:n
        x_1 = [x(1:i-1,:);x(i+1:n,:)];
        y_1 = [y(1:i-1);y(i+1:n)];
        model_1 = OK_Rmodel_kd_nugget(x_1,y_1,0,2);
        [y_1, s_1] = OK_Rpredict(model_1,x(i,:),0,y_1);
        sigma = (y(i)-y_1)/sqrt(s_1);
        if abs(sigma)>alpha_quatile
            f(i) = 1;
        end
    end 
end