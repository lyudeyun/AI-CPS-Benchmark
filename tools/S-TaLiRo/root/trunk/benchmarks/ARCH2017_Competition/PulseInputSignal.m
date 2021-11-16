% StepInputSignal
%    This is an example of how to define custom input signal generators for
%    S-TaLiRo. This example generates a pulse signal parameterized by :
%       1. the initial value of signal x(1),
%       2. the first time that the signal changes value x(2), and 
%       3. the value of signal in the middle x(3), and
%       2. the second time that the signal changes value x(4), and 
%       3. the final value of signal x(5).
%
%    Inputs: 
%       x          - the values of the search variables as provided by S-Taliro
%       timeStamps - the points in time where the values of the signal are
%                    required
%    Output:
%       y - the signal values at the times in timeStamps
%
% See also: ComputeInputSignals

function y = PulseInputSignal(x,timeStamps)

ns = length(timeStamps);
y = zeros(ns,1);
y(timeStamps<x(2)) = x(1);
y((timeStamps>=x(2) & timeStamps<x(4))) = x(3);
y(timeStamps>=x(4)) = x(5);
end