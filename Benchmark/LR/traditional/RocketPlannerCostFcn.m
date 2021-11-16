function J = RocketPlannerCostFcn(stage,x,u)
% Rocket planner cost function.

% Copyright 2020 The MathWorks, Inc.

J = sum(u);
