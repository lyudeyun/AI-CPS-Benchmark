function [T XT YT LT CLG Guards] = BlackBoxPowertrain03(X0,TS, steptime, inputSignal)

% This runs BlackBoxPowertrain02, but with the second input fixed to 0. The
% second input is the road grade. So the only control signal is the
% throttle.
% This was created for staliro_demo_conformance.

% This system takes in a constant input value. So we extract the first value of the input signal and supply that to it.
% Argument inputSignal is assumed to be constant.
X = [inputSignal(1), 0];
% [T XT YT LT CLG Guards]  = BlackBoxPowertrain02(X,TS, steptime);
[T XT YT LT CLG Guards]  = BlackBoxPowertrain02(X,TS);
% The output of the system equals its state
YT = XT(:,[2 5]);  % YT(:,2) is speed, YT(:,5) is engine speed