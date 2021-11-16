function [amp] = cc2amp(cc)
%CC2AMP Calculates the amplitudes from a cycle count.
%
% CALL:  amp = cc2amp(cc);
%
%   amp = a vector with amplitudes.
%   cc    = a two column matrix with cycles.
%
% The amplitude of a cycle is defined as  (Max-min)/2
%
% Example:
%   x=load('sea.dat'); 
%   TP = dat2tp(x);
%   [mM,Mm] = tp2mm(TP);
%   amp = cc2amp(mM);
%   histgrm(amp);
%
%   assert(size(amp),[1086, 1], eps);
%
%   close all;
%
% See also  dat2tp, tp2rfc, tp2mm

% Tested on Matlab 6.0
%
% History:
% Revised by jr 01-Apr-2001
% - histo-> histgrm in example
% - minor changes help text
% Corrected by PJ 09-Dec-1999
%   Now calculates amplitudes (before ranges)
% Created by PJ (Par Johannesson) 01-Nov-1999

% Check input arguments

ni = nargin;

%error(nargchk(1,1,ni));
narginchk(1,1)
% Calculate amplitudes

amp = (cc(:,2)-cc(:,1))/2;
%amp = abs(cc(:,2)-cc(:,1));
