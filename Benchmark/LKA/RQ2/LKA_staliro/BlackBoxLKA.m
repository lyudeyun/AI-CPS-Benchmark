function [T, XT, YT,LT, CLG, Guards] = BlackBoxLKA(X0,simT,TU,U)
% script created for S-Taliro to simulate its input model 
% inputs:
%       X0: systems initial condition or []
%       simT: simulation time
%       TU: input time vector
%       U: input signal
% relevant outputs:
%       T: time sequence
%       XT: system states
%       YT: system outputs
%
% See also: staliro_options, Apply_Opt_GD_default, GdDoc
global cur_mdl agent Ts  

global opt trials max_obj_eval;

global e1_initial  e2_initial Vx;


LT = [];
CLG = [];
Guards = [];

Ts = 0.1;

m = 1575;   % total vehicle mass (kg)
Iz = 2875;  % yaw moment of inertia (mNs^2)
lf = 1.2;   % longitudinal distance from center of gravity to front tires (m)
lr = 1.6;   % longitudinal distance from center of gravity to rear tires (m)
Cf = 19000; % cornering stiffness of front tires (N/rad)
Cr = 33000; % cornering stiffness of rear tires (N/rad)
 

u_min = -0.5;   % maximum steering angle
u_max = 0.5;    % minimum steering angle

line_width = 3.7;   % highway lane width
avg_car_width = 2;  % average car width
max_late_dev = (line_width-avg_car_width)/2-0.1;
max_rel_yaw_ang = 0.261799; % lateral deviation tolerence
terminate_error = 1.5;

rho = 0.001;        %  curvature of the road

time = 0:Ts:15;




Vx = X0(1);          % Roll angle from wings level (rad)
e1_initial = X0(2);        % Pitch angle from nose level (rad)
e2_initial = X0(3);          % Yaw angle from North (rad)     


md = getCurvature(Vx,time);
% MPC parameters
PredictionHorizon = 10;     % MPC prediction horizon



% Change the parameter values in the model
% set_param([model,'/Pedal Angle (deg)'],'Amplitude',num2str(X0(1)));
% set_param([model,'/Pedal Angle (deg)'],'Period',num2str(X0(2)));




% Run the model
simopt = simget(cur_mdl);
simopt = simset(simopt,'SaveFormat','Array'); % Replace input outputs with structures
[T, XT, YT] = sim(cur_mdl);
% mdlWks = get_param(mdl,'ModelWorkspace');
% assignin(mdlWks,'T',TU')
% assignin(mdlWks,'U',U')
% simopt = simget(mdl);
% simset(simopt,'SaveFormat','Array'); % Replace input outputs with structures  % ,'MaxStep', 0.1
% [T, XT, YT] = sim(mdl,[0 simT]);
end


 
 


 