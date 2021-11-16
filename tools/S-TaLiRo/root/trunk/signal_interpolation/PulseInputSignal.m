% PulseInputSignal
%    It generates pulse input signals.
%
%    Inputs: 
%       x - a 6 dim vector with the values of the search variables as 
%           provided by S-Taliro
%             x(1) - the time that the pulse signal starts
%             x(2) - the period of phase 1
%             x(3) - the period of phase 2
%             x(4) - the value of the signal before the pulse starts
%             x(5) - the value of the signal at phase 1
%             x(6) - the value of the signal at phase 2
%       t - the points in time where the values of the signal are requested
%
%    Output:
%       y - the signal values at the times in vector t
%
% See also: ComputeInputSignals

% (C) G. Fainekos

function yy = PulseInputSignal(xx,tt)

assert(isvector(xx)&&length(xx)==6,' StepInputSignal : vector x must be of length 6.')

nt = length(tt);
tshift = xx(1)+xx(2)+xx(3);
yy = zeros(nt,1);
yy(tt<xx(1)) = xx(4);
yy(xx(1)<=tt & tt<xx(1)+xx(2)) = xx(5);
yy(xx(1)+xx(2)<=tt & tt<tshift) = xx(6);
while(tshift<=tt(end))
    yy(tshift<=tt & tt<tshift+xx(2)) = xx(5);
    yy(tshift+xx(2)<=tt & tt<tshift+xx(2)+xx(3)) = xx(6);
    tshift = tshift+xx(2)+xx(3);
end

end