function [ei_0, s_0, y_0] = EIcalc_kd(x_0,x,M_model,y) % EI calculator

[curr_best, curr_best_ind] =  min(y);

b_0 = ones(size(x_0,1),1); %design for constant mean regression?(vector of 1's as b)
[y_0,s_0] = OK_Rpredict(M_model,x_0,0,y); 

i=1;
found=0;
while (i<=size(x_0,1) && found==0)
    if x_0(i,:) == x(curr_best_ind,:)
        curr_best = y_0(i);
        found=1;
    else
        i=i+1;
    end
end
counts = size(x_0,1);
ei_0 = zeros(size(x_0,1),1);
s_0 = sqrt(s_0);

for i = 1:counts
    if s_0(i)>0
        ei_0(i) = (curr_best-y_0(i)) * normcdf((curr_best-y_0(i))/s_0(i),0,1) + s_0(i) * normpdf((curr_best-y_0(i))/s_0(i),0,1);
    end
    if s_0(i)<=0
        ei_0(i)=0;
    end
    
end

end