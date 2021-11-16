function mrl=reslife(data,varargin)
%RESLIFE Mean Residual Life, i.e., mean excesses vs thresholds
%
% CALL  mrl = reslife(data,options)
%
% mrl     = Mean residual life values, i.e., mean excesses over thresholds, u.
%
% data    = vector of data.
% options = options structure defining estimation of mrl. 
%        .u       :  threshold values (default linspace(umin,umax,Nu))
%        .Nmin : Minimum number of extremes to include. (Default Nmin = 3).
%        .umin : Minimum threshold (default min(data))
%        .umax : Maximum threshold (default max(data))
%        .Nu   : number of threshold values (default min(length(data),100))
%        .alpha: Confidence coefficient (default 0.05)
%        .plotflag:h
%
% RESLIFE estimate mean excesses over thresholds. The purpose of MRL is
% to determine the threshold where the upper tail of the data can be 
% approximated with the generalized Pareto distribution (GPD). The GPD is
% appropriate for the tail, if the MRL is a linear function of the 
% threshold, u. Theoretically in the GPD model 
%
%   E(X-u0|X>u0) = s0/(1+k)
%   E(X-u |X>u)  = s/(1+k) = (s0 -k*u)/(1+k)   for u>u0
%
% where k,s is the shape and scale parameter, respectively.
% s0 = scale parameter for threshold u0<u.
%
% Example
%  opt = reslife('defaults'); % Get default options
%  opt.Nu = 20;
%  R = rndgenpar(0.2,2,2,100,1);
%  mrl = reslife(R,opt);
%  plot(mrl);
%
%  close all;
%  
% See also plotreslife, fitgenpar, fitgenparrange, disprsnidx

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


options = struct('u',[],'umin',[],'umax',[],'Nu',100,'Nmin',3,'alpha',0.05,'plotflag',0);

if ischar(data) && strcmpi(data,'defaults')
  mrl = options;
  return
end
options = parseoptions(options,varargin{:});
if isempty(options.u)
sd = sort(data);
n = length(data);

if isempty(options.Nmin)
   options.Nmin=3;
else
  options.Nmin = max(options.Nmin,0);
end
if options.Nmin>n/2
  warning('WAFO:RESLIFE','Nmin possibly too large!')
end

sdmax = sd(n-options.Nmin);
if isempty(options.umax)
  options.umax = sdmax;
else
  options.umax = min(options.umax,sdmax);
end

if isempty(options.umin)
  options.umin = sd(1);
else
  options.umin = max(options.umin,sd(1));
end


if isempty(options.Nu)
  options.Nu = min(n-options.Nmin,100);
end
u = linspace(options.umin,options.umax,options.Nu).';
else
  u = options.u(:);
end

Nu = length(u);
nan1 = nan;
mrl1 = nan1(ones(Nu,1));
srl = mrl1;
num = mrl1;

for ix=1:Nu;
  [mrl1(ix),srl(ix),num(ix)] = myMeanAndStd(data(data>u(ix))-u(ix));
end
p = 1-options.alpha;
alpha2 = options.alpha/2;

% Approximate P% confidence interval
%Za = -invnorm(alpha2);   % known mean
Za   = -invt(alpha2,num-1); % unknown mean
mrlu = mrl1 + Za.*srl./sqrt(num);
mrll = mrl1 - Za.*srl./sqrt(num);

options.CI = [mrll,mrlu];
options.numdata = num;
titleTxt = sprintf('Mean residual life with %d%s CI',100*p,'%');
mrl = createwdata('data',mrl1,'args',u,...
'dataCI',options.CI,'title',titleTxt,'labels',{'Threshold','Mean Excess'},...
    'workspace',options,'note',titleTxt);
  
mrl = wdata(mrl);
if options.plotflag>0
  plot(mrl,options.plotflag,'.')
end

% ps2 = 'r--';
% plot(u,mrl1,'.',u,mrlu,ps2,u,mrll,ps2)
% xlabel('Threshold');
% ylabel('Mean Excess');
% title('Mean residual life')


function [m,s,n] = myMeanAndStd(dat1)
n = length(dat1);
m = mean(dat1);
s = std(dat1);
