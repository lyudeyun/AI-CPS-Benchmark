% powertrain_shift_speed_12 computes the speed for shifting from 1st to 2nd 
% gear for the Powertrain_CheckMate_ver Simulink model.
%
% Usage:
% V = powertrain_shift_speed_12(tps)
% 
% Input: tps - throttle position in %
% Output: V - velocity

% G. Fainekos - ASU

function out = powertrain_shift_speed_21(tps)

% In km/hr
if tps <= 80
    out = 14;
elseif (80<tps) && (tps<80.1)
    out = 364*(tps-80)+14;
else
    out = 50.4;
end

% In m/s
out = out/3.6;

end
