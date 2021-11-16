% powertrain_shift_speed_12 computes the speed for shifting from 1st to 2nd 
% gear for the Powertrain_CheckMate_ver Simulink model.
%
% Usage:
% V = powertrain_shift_speed_12(tps)
% 
% Input: tps - throttle position in %
% Output: V - velocity

% G. Fainekos - ASU

function out = powertrain_shift_speed_12(tps)

% In km/h
if tps <= 30
    out = 20;
elseif (30<tps) && (tps<80)
    out = 0.7*(tps-30)+20;
else
    out = 55;
end

% In m/s
out = out/3.6;

end
