function [e,m,sigma2]= plotsn(S,N,nr)
%PLOTSN Plots SN-data and estimate parameters 
%       according to the least-squares method
%
% CALL:  [e,beta,s2] = plotsn(S,N,nr);
%
%  where
%
%        e,beta = the parameters in the model, 
%        s2     = the residual variance for linear regression 
%                 in logS/logN plane,
%        S      = a nx1 vector with S-data,
%        N      = a nx1 vector with N-data,
%        nr     = plot parameter (optional input argument);
%
%                 1 = only SN-data will be plotted,
%                 2 = SN-data and fitted line will be plotted,
%                 3 = only log(S)/log(N)-data will be plotted,
%                 4 = log(S)/log(N)-data and fitted line will be plotted,
%                11-14 = Same as above but x-axis = N, y-axis = S . 
%  
% Model:
%       N(s) = K/(e*s^beta)
%
% Example:  
%   sn = load('sn.dat'); s = sn(:,1); N = sn(:,2);
%   [e,beta,s2] = plotsn(s,N,2);   % S-N, x-axis = S, y-axis = N
%   [e,beta,s2] = plotsn(s,N,12);  % N-S, x-axis = N, y-axis = S
%
%   close all;
  
% Tested on: Matlab 6.0
% History:
% Correction by PJ 07-Jul-2005
%   Changed 'break' to 'return'
% Revised by jr 01-Apr-2001
% - example, help text
% Revised by PJ 10-Jan-2000
%   updated for WAFO
% Original version from FAT by Mats Frendahl 
%   Copyright 1993, Mats Frendahl, Dept. of Math. Stat., University of Lund.

[n,m] = size(S);
if m>n, S=S'; end
[n,m] = size(S);
if m~=1, disp('   First argument must be  nx1, program will terminate.'); end

[n,m]=size(N);
if m>n, N=N'; end
[n,m]=size(N);
if m~=1, disp('   Second argument must be  nx1, program will terminate.'); end

A = [ones(length(S),1) log(S)];
beta = inv(A'*A)*A'*log(N); 
m = -beta(2); 
e = exp(-beta(1));
Q0 = log(N)'*log(N)-beta'*A'*log(N);
sigma2 = Q0/(length(log(S))-2);


if nargin==3
  k = exp(sigma2/2);
  borderS = [min(S) max(S)]; 
  borderN = [min(N) max(N)]; 
  number_of_s = 99;

  s = borderS(1):(borderS(2)-borderS(1))/number_of_s:borderS(2);
  n = k / e * s.^(-m);
  if nr<10
    if nr==1 || nr==3
     plot(S, N, '.', 'markersize', 12);
     title('SN-data');
    elseif nr==2 || nr==4
      plot(s, n,'-', S, N, '.', 'markersize', 12);
      title('SN-data with estimated N(s)');
    end
    
    axis([[0.9 1.1].*borderS 0 1.1*borderN(2)]);
    xlabel('s');
    ylabel('N(s)');
  else
    if nr==11 || nr==13
      plot(N,S,'.','markersize',12);
      title('SN-data');
    elseif nr==12 || nr==14
      plot(n,s,'-',N,S,'.','markersize',12);
      title('SN-data with estimated N(s)');
    end

    axis([0 1.1*borderN(2) [0.9 1.1].*borderS ]);
    xlabel('N');
    ylabel('s');
  end
  
  if mod(nr,10) > 2 % if number == 3,4,13,14, then log-scale
    h=gca();
    set(h,'YScale','log','XScale','log');
  end

end

return

