function [T, XT, YT, LT, CLG, Guards] = BlackBoxWTK(X0,simT,TU,U)
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

global cur_mdl agent h_ref_min h_ref_max;
global opt Ts trials max_obj_eval;


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


 
 


 