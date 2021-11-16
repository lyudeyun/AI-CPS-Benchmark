% scipt created for S-Taliro to search the system parameters 
% Created: 8/30/2018
% the BlackBox interface is fixed.
% inputs:
%       X0: systems initial condition or []
%       simT: simulation time
%       TU: input time vector
%       U: input signal
% relevant outputs:
%       T: time sequence
%       XT: system states
%       YT: system outputs


function [T, XT, YT, LT, CLG, Guards] = BlackBoxNN(X0,simT,TU,U)
LT = [];
CLG = [];
Guards = [];
model = 'steamcondense_RNN_22';
global simin
simin.time = TU; simin.signals.values = U;
% Run the model
simopt = simget(model);
simopt = simset(simopt,'SaveFormat','Array','MaxStep', 0.1); % Replace input outputs with structures
[T, XT, YT] = sim(model,[0 simT]);
end