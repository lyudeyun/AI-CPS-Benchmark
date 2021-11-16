function[cd_0] = CrowdingDist_kd(x_0,x) 
DimWise_Crowd = NaN(1,size(x_0,2));
cd_0 = NaN(size(x_0,1),1);
    for i = 1:size(x_0,1)
        for j = 1:size(x_0,2)
            DimWise_Crowd(j) = min(abs((ones(size(x,1),1).*x_0(i,j))-x(:,j)));
        end
        cd_0(i,1) = sum(DimWise_Crowd);
    end
end