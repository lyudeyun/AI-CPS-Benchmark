function f = fold5_cv_kd_OK(x, y, alpha_quatile)
%alpha_quatile corresponds to the number that yeidls the desired quantile
%of a standard normal distribution i.e. 95% ==> 1.97
    
    n = size(x,1);
    fs = floor(n/5);
    f = zeros(5,1);
 
    for i = 1:5
        bot = (i-1)*fs; 
        if i == 5
            top = n+1;
        else
            top = i*fs+1;
        end
        x_1 = [x(1:bot,:);x(top:n,:)];
        y_1 = [y(1:bot);y(top:n)];
        
        model_1 = OK_Rmodel_kd_nugget(x_1,y_1,0,2);
        
        [y_p, s_1] = OK_Rpredict(model_1,x(bot+1:top-1,:),0,y_1);
        
        sigma = (y(bot+1:top-1)-y_p)./sqrt(s_1);
        
        if any(abs(sigma)>alpha_quatile)
            f(i) = 1;
        end
    end 
end