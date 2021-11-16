function [m,v,sk,ku]= momgumb(varargin)
%MOMGUMB Mean and variance for the Gumbel distribution.
% 
% CALL:  [m,v,sk,ku] = momgumb(a,b,options)
%
%   m, v = the mean and variance, respectively.
%  sk,ku = the skewness and kurtosis, respectively. 
%   a, b = parameters of the Gumbel distribution (see cdfgumb)
%  options = options struct with fieldnames:
%   trunc = 0  regular gumbel distribution (default)
%           1  truncated gumbel distribution (not available)
%
%  Mean (m) and variance (v) for the Gumbel distribution is
%
%  m=Euler*a+b  and  v=(a*pi)^2/6  where Euler is Euler's
%  constant 0.5772...
%
% Example:
%   par = {5,10};
%   X = rndgumb(par{:},10000,1);
%   moments = {mean(X) var(X),skew(X),kurt(X)};   % Estimated mean and variance
%   [mom{1:4}] = momgumb(par{:}); % True mean and variance
%   assert(moments, mom, -0.25);
%
% See also  pdfgumb, cdfgumb, invgumb, rndgumb, fitgumb,

%
%     This program is free software; you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published by
%     the Free Software Foundation; either version 3 of the License, or
%     (at your option) any later version.
% 
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU General Public License for more details.
% 
%     You should have received a copy of the GNU Lesser General Public License
%     along with this program.  If not, see <http://www.gnu.org/licenses/>.


% Reference: 
%  Johnson  N.L., Kotz S. and Balakrishnan, N. (1994)
%  Continuous Univariate Distributions, Volume 2. Wiley. 


%  tested on: matlab 5.2
% history
% revised pab 8.11.1999
% updated header info
%   Per A. Brodtkorb 17.10.98
% revised ms 14.06.2000
% - changed name to momgumb (from gumbstat)
% - revised header info
% - noted that calculations are wrong for trunc=1 (not corrected)

% TODO % calculation wrong for trunc=1
%error(nargchk(1,inf,nargin))
narginchk(1,inf)
Np = 2;
options = struct('trunc',false); % default options
[params,options] = parsestatsinput(Np,options,varargin{:});
if numel(options)>1
  error('Multidimensional struct of distribution parameter not allowed!')
end
[a,b] = deal(params{:});
if isempty(b)
  error('Location parameter b undefined!')
end

a(a<=0) = nan;
try
  m = 0.577215664901532*a+b; %mean
  v = pi^2/6*a.^2; %variance
  
  zeta3 = 1.2020569; % apery's constant
  sk  = 12*sqrt(6)*zeta3/pi^3;
  ku = 3+12/5;
  
  if options.trunc, %This is not correct (ms)
    warning('WAFO:MOMGUMB','mean and variance not correct!')
    tmp = -expm1(-exp( b./a));
    m=m./tmp;
    v=v./tmp;
  end
  [csz,v,sk,ku]=comnsize(v,sk,ku,m);
catch
  error('a and b  must be of common size or scalar.');
end



