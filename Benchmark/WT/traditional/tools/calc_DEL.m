function DEL = calc_DEL(time,signal)
% CALC_DEL    Calculate damage equivalent load using WAFO.
%    CALC_DEL(TIME,SIGNAL) is the damage equivalent load for SIGNAL over
%    TIME.  This script requires WAFO 2.1.1 to be in the path.
%
% Eric Westervelt
% 18 Sep 2008

T_sim = time(end); %length of time series in seconds
f_ref = 0.809;  % reference frequency
slope = 4;      % this slope corresponds to the material (4 for tower, 10 for blades)
Nref = T_sim*f_ref;

TBMt = [time signal];
tp   = dat2tp(TBMt);
rfc  = tp2rfc(tp);
dam  = cc2dam(rfc,slope);

% damage equivalent loads for peak-to-peak amplitudes
DEL = 2*(dam/Nref)^(1/slope);
