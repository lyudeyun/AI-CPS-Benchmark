function [x1, incr, is_last_pnt] = get_next_point(x0,delta,inpRanges, io)
% SYNOPSYS
%   [x1, incr] = get_next_point(x0,delta,inpRanges, io)
% 
% DESCRIPTION
%   Given a base point x0, get next one in n-dimensiontal rectangle
%   inpRanges, differing by an increment of delta along the last
%   dimension in inpRanges that can still increase.
%   Called in a loop to systematically sample inpRanges in increments of 
%   delta.
%   
% INPUTS
% - x0 : n-by-1 starting point, from which the next will be chosen
% - delta: scalar increment along any one dimension
% - inpRanges: n-by-2, gives lower and upper boundaries of rectagle, so
% returned point x1 satisfies inpRanges(i,1) <= x1(i) <= inpRanges(i,2)
% - io: iteration order, optional. Gives order in which the dimensions will be
% incremented. Default order is from last to first. If io
% contains fewer entries than x0, then only those dimensions specified in
% it will be incremented. E.g. if x0 = [x1 x2 x3], this command
%   get_next_point([x1 x2 x3], delta,inpRanges, [2,1])
% is equivalent to
%   xnext = get_next_point([x1 x2], delta,inpRanges([1,2],:))
%   xnext = [xnext x3];
% 
% OUTPUTS
% - x1: next point
% - incr: 1 if fnt actually incremented vetor, 0 otherwise (which happens in
% case input x0 is already at boundary). Can be used as stopping criterion
% of sampling in caller fnt. If incr=0, x1=x0.
% - is_last_pnt: vector of length n. is_last_pnt(i)=1 indicates the iterator
% has reached last pnt possible along the ith dimension.
% 
% EXAMPLE
%
% incr=1; x0=zeros(1,2);delta=1;inpRanges=[0 2;0 2];
% while(incr)
% [x1 incr]=get_next_point(x0,delta,inpRanges);
% disp(num2str([x1 incr]));
% x0=x1;
% end
% 0  1  1
% 0  2  1
% 1  0  1
% 1  1  1
% 1  2  1
% 2  0  1
% 2  1  1
% 2  2  1
% 2  2  0


n=size(inpRanges,1);
if nargin < 4
    io = 1:n;
end    

assert(n==length(x0));
if length(x0)==1
    if x0 >= inpRanges
        incr = 0;
        x1 = x0;
    else
        x1 = x0+delta;
        incr= 1;
    end
elseif isempty(x0)
   incr = 0;
   x1 = x0;
else
    [y1, incr] = get_next_point(x0(io(2:end)),delta,inpRanges(io(2:end),:));
    if incr
        x1 = x0; x1(io(2:end)) = y1;        
    else
        [y1,incr]=get_next_point(x0(io(1)),delta,inpRanges(io(1),2));
        if incr
            x1 = x0; 
            x1(io(1)) = y1; 
            x1(io(2:end)) = inpRanges(io(2:end),1);            
        else
            x1=x0;
            x1(io(1)) = y1;
            x1(io(2:end)) = x0(io(2:end));
        end
    end
end

if nargout == 3
    is_last_pnt = x1==inpRanges(:,2)';    
end