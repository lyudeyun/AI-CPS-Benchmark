% StepInputSignal
%    This is an example of how to define custom input signal generators for
%    S-TaLiRo. This example generates a step signal parameterized by :
%       1. the value of the signal before the step x(1),
%       2. the time that the signal changes value x(2), and 
%       3. the value of the signal after the step x(3).
%
%    Inputs: 
%       x          - the values of the search variables as provided by S-Taliro
%       timeStamps - the points in time where the values of the signal are
%                    required
%    Output:
%       y - the signal values at the times in timeStamps
%
% See also: ComputeInputSignals

% (C) G. Fainekos

function y = StepInputSignal(x,timeStamps)

assert(isvector(x)&&length(x)==3,' StepInputSignal : vector x must be of length 3.')

ns = length(timeStamps);
y = zeros(ns,1);
y(timeStamps<x(2)) = x(1);
y(timeStamps>=x(2)) = x(3);

end