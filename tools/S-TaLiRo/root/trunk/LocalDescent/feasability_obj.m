function [f,g] = feasability_obj(z,commonargs)
% Return F(z) = objective function, and gradient at z = [0 ... 0 1]
D = length(z);
g = [zeros(1,D-1) 1];
f = z(end);
