% plot_powertrain plots the state variables and the current gear for the
% Powertrain_CheckMate_ver Simulink model.
%
% Usage:
% plot_powertrain(T,[XT LT])
%

% G. Fainekos - ASU

function plot_powertrain(tout,yout)

figure

subplot(2,3,1)
plot(tout,yout(:,1))
grid on
title('T_s')

subplot(2,3,2)
plot(tout,yout(:,2))
grid on
title('veh speed')

subplot(2,3,3)
plot(tout,yout(:,3))
grid on
title('pc_{2,filter}')

subplot(2,3,4)
plot(tout,yout(:,4))
grid on
title('\omega_t')

subplot(2,3,5)
plot(tout,yout(:,5))
grid on
title('\omega_{cr}')

subplot(2,3,6)
plot(tout,yout(:,end))
% axis([0 60 0 3])
grid on
title('location')

end