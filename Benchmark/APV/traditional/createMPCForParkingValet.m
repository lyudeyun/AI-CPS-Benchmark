function [nlobj,opt,paras] = createMPCForParkingValet(p,c,Ts,egoInitialPose,egoTargetPose,maxIter,...
                                                Qp,Rp,Qt,Rt,distToCenter,safetyDistance,midPoint)
% This function create a nlmpc object for parking. 

% Copyright 2017-2019 The MathWorks, Inc.

%%
paras = {egoTargetPose,Qp,Rp,Qt,Rt,distToCenter,safetyDistance}';

%%
% mpc initialization
nx = 3;
ny = 3;
nu = 2;
nlobj = nlmpc(nx,ny,nu);

% vehicle dynamics
nlobj.Model.StateFcn = "parkingVehicleStateFcn";
nlobj.Jacobian.StateFcn = "parkingVehicleStateJacobianFcn";

% mpc settings
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = c;
nlobj.Model.NumberOfParameters = numel(paras);

% mpc limits
nlobj.MV(1).Min = -6.5;
nlobj.MV(1).Max = 6.5;
nlobj.MV(2).Min = -pi/4;
nlobj.MV(2).Max = pi/4;

% mpc cost function
nlobj.Optimization.CustomCostFcn = "parkingCostFcn";
nlobj.Optimization.ReplaceStandardCost = true;
nlobj.Jacobian.CustomCostFcn = "parkingCostJacobian";

% mpc constraints
nlobj.Optimization.CustomIneqConFcn = "parkingValetIneqConFcn";
nlobj.Jacobian.CustomIneqConFcn = "parkingValetIneqConFcnJacobian";


% mpc optimization solver options.
nlobj.Optimization.SolverOptions.FunctionTolerance = 0.01;
nlobj.Optimization.SolverOptions.StepTolerance = 0.01;
nlobj.Optimization.SolverOptions.ConstraintTolerance = 0.01;
nlobj.Optimization.SolverOptions.OptimalityTolerance = 0.01;
nlobj.Optimization.SolverOptions.MaxIter = maxIter;

% mpc initial guess.
opt = nlmpcmoveopt;
xGuess = [linspace(egoInitialPose(1),midPoint(1),p/2),...
    linspace(midPoint(1),egoTargetPose(1),p/2)]';
yGuess = [linspace(egoInitialPose(2),midPoint(2),p/2),...
    linspace(midPoint(2),egoTargetPose(2),p/2)]';
yawGuess = [linspace(egoInitialPose(3),midPoint(3),p/2),...
    linspace(midPoint(3),egoTargetPose(3),p/2)]';
opt.X0 = [xGuess,yGuess,yawGuess];
opt.MV0 = zeros(p,nu); 
opt.Parameters = paras;

% mpc validate Jacobians on random x0 and u0. Note that Jacobian of
% inequality constraint will give a warning since it is an approximation
% and not exact. Uncomment the following two lines for validation.
% rng(100);
% validateFcns(nlobj,randn(3,1),randn(2,1),[],paras);

end

