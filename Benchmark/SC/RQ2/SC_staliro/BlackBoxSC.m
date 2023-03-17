function [T, XT, YT, LT, CLG, Guards] = BlackBoxSC(X0,simT,TU,U)
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

global cur_mdl  agent Ts min_Fs max_Fs P_step P_init P_final time_tol;


LT = [];
CLG = [];
Guards = [];


% load_system(mdl);



% Ts = 0.1;
% min_Fs = 3.99;
% max_Fs = 4.01;      % maximum steam flowrate
% % RL parameters
% P_step = 2;         % ref pressure step time
% P_init = 90;        % initial ref pressure
% P_final = 87;       % final ref pressure
% P_terminate = 100;    % error to terminate episode
% time_tol = 3;

% sim(mdl);



  
 

 

% Change the parameter values in the model
% set_param([model,'/Pedal Angle (deg)'],'Amplitude',num2str(X0(1)));
% set_param([model,'/Pedal Angle (deg)'],'Period',num2str(X0(2)));

% Run the model
% simopt = simget(model);
% simopt = simset(simopt,'SaveFormat','Array'); % Replace input outputs with structures
% [T, XT, YT] = sim(model,[0 simT],simopt,[TU U]);
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


 
 


 