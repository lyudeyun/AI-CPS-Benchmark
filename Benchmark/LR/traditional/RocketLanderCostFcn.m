function J = RocketLanderCostFcn(stage,x,u,dmv,p)
% Rocket lander cost function.

% Copyright 2020 The MathWorks, Inc.

if stage == 1
    J = dmv'*[0.1 0;0 0.1]*dmv;
elseif stage == 11
    J = (x-p)'*(x-p);
else
    J = (x-p)'*(x-p) + dmv'*[stage*0.1 0;0 stage*0.1]*dmv;
end