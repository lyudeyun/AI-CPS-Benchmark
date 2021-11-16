 % scipt created for S-Taliro to search the system parameters 

% Xiaoqing Jin
% Created: 9/19/2013

% the BlackBox interface is fixed. 
function [T, XT, YT, LT, CLG, Guards] = BlackBoxAbstractFuelControl(X0,simT,TU,U)

LT = [];
CLG = [];
Guards = [];

model = 'AbstractFuelControl_M1';

% Change the parameter values in the model
% set_param([model,'/Pedal Angle (deg)'],'Amplitude',num2str(X0(1)));
% set_param([model,'/Pedal Angle (deg)'],'Period',num2str(X0(2)));

% Run the model
simopt = simget(model);
simopt = simset(simopt,'SaveFormat','Array'); % Replace input outputs with structures
[T, XT, YT] = sim(model,[0 simT],simopt,[TU U]);

end