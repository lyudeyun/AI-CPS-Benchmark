function nlobjTracking = createMPCForTrackingParkingValet(pTracking,Xref)
% create nlmpc object for tracking.

% Copyright 2019 The Mathworks Inc

%%
mpcverbosity('off');

% mpc initialization
nx = 3;
ny = 3;
nu = 2;
nlobjTracking = nlmpc(nx,ny,nu);

% vehicle dynamics
nlobjTracking.Model.StateFcn = "parkingVehicleStateFcn";
nlobjTracking.Jacobian.StateFcn = "parkingVehicleStateJacobianFcn";

% mpc settings
Ts = 0.1;
nlobjTracking.Ts = Ts;
nlobjTracking.PredictionHorizon = pTracking;
nlobjTracking.ControlHorizon = pTracking;

% mpc limits
nlobjTracking.MV(1).Min = -6.5;
nlobjTracking.MV(1).Max = 6.5;
nlobjTracking.MV(2).Min = -pi/4;
nlobjTracking.MV(2).Max = pi/4;

% mpc weights
nlobjTracking.Weights.OutputVariables = [1,1,3]; 
nlobjTracking.Weights.ManipulatedVariablesRate = [0.1,0.05];

% mpc terminal condition on U
nlobjTracking.Optimization.CustomEqConFcn = @(X,U,data) (U(end,:)-[0,0])';


% mpc nloptions
nloptions = nlmpcmoveopt;
nloptions.X0 = Xref(2:pTracking+1,:);
end