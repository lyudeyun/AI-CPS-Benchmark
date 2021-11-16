function  f = pdfhyge(x,varargin)
%PDFHYGE  Hypergeometric probability mass function
%
%  CALL f = pdfhyge(x,n,K,N)
%
%       f = probability
%       x = Number of items belonging to class K (x = 0,1,2,...min(n,K))
%       n = sample size without replacement (n<=N)
%       K = total population of item K      (K<=N)
%       N = total population
%
%   The Hypergeometric distribution is defined by its pdf
% 
%   f(x;n,K,N) = K!/(K-x)!/x!*(N-K)!/(N-K-n+x)!/(n-x)! (N-n)!*n!/N!
%
% Example
%  n = 10; K = 30; N = 100;
%  x = 0:min(n,K);
%  f = pdfhyge(x,n,K,N);
%
%   % When n/N is small one can use the binomial approximation for the pdf
%  f1 = pdfbin(x,n,K/N);
%  subplot(2,1,1);bar(x,f);
%  subplot(2,1,2);bar(x,f1);
%
%  semilogy(x,abs(f1-f));
%
%  close all;
%
% See also cdfhyge, invhyge, rndhyge, fithyge, momhyge, binom

%       Anders Holtsberg, 18-11-93
%       Copyright (c) Anders Holtsberg
%
% This program is free software; you can redistribute it and/or modify
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



options = struct('logp',false); % default options
if ischar(x) && strcmpi(x,'defaults')
  f = options;
  return
end

%error(nargchk(2,inf,nargin))
narginchk(2,inf)
Np = 3;

[params,options] = parsestatsinput(Np,options,varargin{:});
if numel(options)>1
  error('Multidimensional struct of distribution parameter not allowed!')
end

[n,K,N] = deal(params{:});


try
  ok = (n<=N & K<=N & 0<=K);
  % if any(any( n>N | K>N | K<0 ));
  %    error('Incompatible input arguments');
  % end

  z = (0>x | 0>n-x | x>K | n-x>N-K);
catch
   error('x, n, K, and N must be of common size or scalar.');
end

i = find(~z & ok);
if any(z)
  if length(x)>1, x = x(i); end
  if length(K)>1, K = K(i); end
  if length(n)>1, n = n(i); end
  if length(N)>1, N = N(i); end
end
f = zeros(size(z));

logp = true;  

if 1
  % Better than using binom directly especially for large N.
  p0  = n/N;
  p1 = pdfbin(x,K,p0,'logp',logp);
  p2 = pdfbin(n-x,N-K,p0,'logp',logp) ;
  p3 = pdfbin(n,N,p0,'logp',logp);
else
  p1 = binom(K,x,logp);
  p2 = binom(N-K,n-x,logp) ;
  p3 = binom(N,n,logp);
end
pp = p1+p2-p3;
if options.logp
  f(:) = -inf;
else
  pp = exp(pp);
end

f(i) = pp(:);
%f(~isfinite(x) | ~ok) = nan;