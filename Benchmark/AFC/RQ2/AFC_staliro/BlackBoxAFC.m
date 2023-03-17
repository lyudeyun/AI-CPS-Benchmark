function [T, XT, YT, LT, CLG, Guards] = BlackBoxAFC(X0,simT,TU,U)
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
global cur_mdl agent Ts fuel_inj_tol MAF_sensor_tol AF_sensor_tol pump_tol kappa_tol tau_ww_tol fault_time kp ki;
global min_Engine_Speed max_Engine_Speed min_Pedal_Angle max_Pedal_Angle;
global opt trials max_obj_eval;


LT = [];
CLG = [];
Guards = [];

% Change the parameter values in the model
% set_param([model,'/Pedal Angle (deg)'],'Amplitude',num2str(X0(1)));
% set_param([model,'/Pedal Angle (deg)'],'Period',num2str(X0(2)));

% Run the model
simopt = simget(cur_mdl);
simopt = simset(simopt,'SaveFormat','Array'); % Replace input outputs with structures
[T, XT, YT] = sim(cur_mdl,[0 simT],simopt,[TU U]);
% mdlWks = get_param(mdl,'ModelWorkspace');
% assignin(mdlWks,'T',TU')
% assignin(mdlWks,'U',U')
% simopt = simget(mdl);
% simset(simopt,'SaveFormat','Array'); % Replace input outputs with structures  % ,'MaxStep', 0.1
% [T, XT, YT] = sim(mdl,[0 simT]);
end


 
 


 