function r= polyishft(p,a,b)
% POLYISHFT Inverse polynomial coefficient shift
%
% CALL:  py = polyishft(px,a,b)
%
%   px,py = polynomial coefficients for the X and Y variable, respectively
%   a,b   = lower and upper limits 
%
% POLYISHFT do the inverse of POLYSHFT. 
% POLYSHFT shift the polynomial coefficients by a variable shift:
%
%   Y = 2*(X-.5*(b+a)/(b-a)
%
% i.e., the interval a <= X <= b is mapped to the interval -1 <= Y <= 1
% 
% Example:
%  px = [1 0];
%  py = polyishft(px,-5,5);
%  assert(polyval(px,[-5 0 5]), [-5 0 5], 1e-10);  % This is the same as the line below
%  assert(polyval(py,[-1 0 1 ]), [-5, 0, 5], 1e-10);  
% 
% See also: polyshft

% Reference 
% William H. Press, Saul Teukolsky, 
% William T. Wetterling and Brian P. Flannery (1997)
% "Numerical recipes in Fortran 77", Vol. 1, pp 184-194

% History
% by pab 2000

%error(nargchk(3,3,nargin));
narginchk(3,3)
r = polyshft(p,-(2+b+a)/(b-a),(2-b-a)/(b-a));










