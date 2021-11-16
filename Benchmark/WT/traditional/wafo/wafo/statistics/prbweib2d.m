function  [P ,err] = prbweib2d(x1lo,x1up,x2lo,x2up,varargin)
%PRBWEIB2D returns the probability for rectangular regions.
%
% CALL: [P, err] = prbweib2d(x1lo,x1up,x2lo,x2up,A1,B1,A2,B2,C12,options);
%       [P, err] = prbweib2d(x1lo,x1up,x2lo,x2up,phat,options);
%       
%   P    = probability
%   err  = absolute tolerance, i.e., abs(int-intold)
%   xilo = lower integration limits
%   xiup = upper integration limits
%   A1, A2 = scale parameters    
%   B1, B2 = shape parameters
%      C12 = interaction parameter between X1 and X2
%     phat = Distribution parameter struct
%            as returned from FITWEIB2D.  
%  options = struct with fieldnames:
%     .logp    : if TRUE, probability, p, returned as log(p).
%     .releps  : specified relative tolerance (Default 1e-3) 
% 
%  The size of P is the common size of XILO and XIUP.  
% 
% Example
%  x1 = linspace(0,10)';
%  phat = {1 2 .5 1.5 .8};
%  prb = prbweib2d(1,2,1,2,phat{:});
%  f = pdfweib2d(x1,x1,phat{:},'meshgrid',true,'wdata',true);
%  plot(f); hold on,
%  plot([ 1 1 2 2 1],[1 2 2 1 1]); hold off;
%
%  See also  pdfweib2d, cdfweib2d pdfweib, gaussq2d, gaussq

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



% tested on: matlab 5.2
% history:
% revised pab 27.10.2000
%  - added example text
%  Per A. Brodtkorb 28.10.98

%error(nargchk(5,15,nargin))
narginchk(5,15)
Np = 5;
options = struct('covariance',[],'alpha',0.05,...
  'logp',false,'releps',1e-3); % default options
[params,options] = parsestatsinput(Np,options,varargin{:});
if numel(options)>1
  error('Multidimensional struct of distribution parameter not allowed!')
end

[a1,b1,a2 b2,c12] = deal(params{:});

if isempty(a1)||isempty(b1)||isempty(a2)||isempty(b2)||isempty(c12)
  error('Requires either 7 input arguments or that input argument 3 is FDATA.'); 
end

[P, err] = gaussq2d(@pdfweib2d,x1lo,x1up,x2lo,x2up,options.releps,params{:});


