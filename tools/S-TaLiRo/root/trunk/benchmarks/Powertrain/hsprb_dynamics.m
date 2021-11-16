% Switched continuous dynamic function for the MoBIES hybrid system
% analysis challenge problem.
%
% Syntax:
% "[sys,type,reset] = hsprb_dynamics(x,u)"
%
% Description:
% Returns the continuous state derivatives, the type of system dynamics,
% and the reset vector for the continuos states as function of "x", the
% current continuous state vector, and "u", the discrete input vector to 
% the switched continuos system.

% The model is described in detail in the technical report:
% Alongkrit Chutinan and Kenneth R. Butts, Dynamic Analysis of Hybrid 
% System Models for Design Validation
%
% The current modifications so the model can run under CheckMate 3.6 were
% done by:
% Yashwanth Annapureddy and Georgios Fainekos, Arizona State University

function [sys,type,reset] = hsprb_dynamics(x,u)

type = 'nonlinear';
if any(u == 0)
    sys = zeros(6,1);
    reset.A = eye(6);
    reset.B = zeros(6,1);
    return
end

% ================
% Model Parameters
% ================

M = 1644.0 + 125.0; % Vehicle mass (kg)
Hf = 0.310;         % Static ground-to-axle height of front wheel (m)
Iwf = 2.8;          % Front wheel inertia (both sides) (kg-m^2)
Ks = 6742.0;        % Driveshaft spring constant (Nm/rad)

Rsi = 0.2955;       % Input sun gear ratio
Rci = 0.6379;       % Input carrier gear ratio
Rcr = 0.7045;       % Reaction carrier gear ratio
Rd = 0.3521;        % Final drive gear ratio

R1 = Rci*Rsi/(1-Rci*Rcr); % 1st gear ratio
R2 = Rci;                 % 2nd gear ratio

AR2 = 4.125;
c2_mu1 = 0.1316;          % mu2 = c2_mu1 + c2_mu2*fabs(c2slip)
c2_mu2 = 0.0001748;

It = 0.05623;             % Turbine inertia (kg-m^2)
Isi = 0.001020;           % Input sun gear ratio (kg-m^2)
Ici = 0.009020;           % Input carrier gear ratio (kg-m^2)
Icr = 0.005806;           % Reaction carrier gear inertia (kg-m^2)

It1 = It + Isi + R1*R1*Icr + R1*R1/R2/R2*Ici;
It2 = It + Ici + R2*R2*Icr + R2*R2/R1/R1*Isi;

Icr12 = Icr + Isi/R1/R1 + Ici/R2/R2;

c2_table.y = [0 1 1 1];

Pc2max = 400.0;            % kPa

% m_s_to_km_h = 3.6;        % conversion factor between m/s and km/h; unitless
pc2_torque_phase = .4;      % ratio of Pc2max to be applied initially as pressure 
                            % offset
                            

% ==============================
% Model Inputs (Assumed Constant)
% ==============================
tps = evalin ('base','tps');
grade = evalin('base','grade');

% =================
% Continuous States
% =================
Ts = x(1);
veh_speed = x(2);
pc2_filter = x(3);
wt = x(4);
wcr = x(5);
% z = x(6);

% ===============
% Discrete Inputs
% ===============
gear_schedule = u(1);
dynamic_mode = u(2);

% Gear schedule state enumerations
FIRST_GEAR = 1;
TRANSITION12_SHIFTING = 2;
SECOND_GEAR = 3;
TRANSITION21_SHIFTING = 4;

% Dynamic mode enumerations.
FIRST = 1;
TORQUE12 = 2;
INERTIA12 = 3;
SECOND = 4;
INERTIA21 = 5;
TORQUE21 = 6;

% ===================================================
% Compute variables that depends on continuous states
% ===================================================

if ismember(dynamic_mode,[FIRST,TORQUE12,TORQUE21])
    wci = R1/R2*wt;
elseif ismember(dynamic_mode,[INERTIA12,INERTIA21])
    wci = 1/R2*wcr;
elseif ismember (dynamic_mode,SECOND)
    wci = wt;
else
    error('hsprb_dynamics: dynamic_mode is not properly initialized.')
end

% =================================================
% Compute variables that depends on discrete states
% =================================================
if ismember(gear_schedule,[FIRST_GEAR,TRANSITION21_SHIFTING])
    to_gear = 1;
elseif ismember(gear_schedule,[SECOND_GEAR,TRANSITION12_SHIFTING])
    to_gear = 2;
else
    error('hsprb_dynamics: gear_schedule is not properly initialized.')
end

% ====================================
% Compute other intermediate variables
% ====================================

% Engine torque
Tt = 1.7*tps+30;

% Clutch pressures.
pc2_target = Pc2max*c2_table.y(to_gear);
pc2 = pc2_filter + pc2_torque_phase*pc2_target;

% Torques
c2slip = wt - wci;
if (c2slip > -0.5) && (c2slip < 0.5)
    sgn2 = 1;
else
    sgn2= sign(c2slip);
end
Tc2 = sgn2*(c2_mu2*abs(c2slip)+c2_mu1)*AR2*pc2;

% =========================
% Compute State Derivatives
% =========================

Ts_dot = Ks*(Rd*wcr-veh_speed/Hf-0.001*Ts);
veh_speed_dot = (Ts/Hf-M*9.81*sin(grade))/(M+2*Iwf/(Hf*Hf));
pc2_filter_dot = -pc2_filter + (1-pc2_torque_phase)*pc2_target;

if ismember(dynamic_mode,FIRST)
    wt_dot = 1/It1*(Tt-R1*Rd*Ts);
elseif ismember(dynamic_mode,[TORQUE12,TORQUE21])
    wt_dot = 1/It1*(Tt-R1*Rd*Ts-(1-R1/R2)*Tc2);
elseif ismember(dynamic_mode,[INERTIA12,INERTIA21])
    wt_dot = 1/It*(Tt-Tc2);
elseif ismember(dynamic_mode,SECOND)
    wt_dot = 1/It2*(Tt-R2*Rd*Ts);
else
    error('hsprb_dynamics: dynamic_mode is not properly initialized.')
end

if ismember(dynamic_mode,[FIRST,TORQUE12,TORQUE21])
    wcr_dot = R1*wt_dot;
elseif ismember(dynamic_mode,[INERTIA12,INERTIA21])
    wcr_dot = 1/Icr12*(Tc2/R2-Rd*Ts);
elseif ismember(dynamic_mode,SECOND)
    wcr_dot = R2*wt_dot;
else
    error('hsprb_dynamics: dynamic_mode is not properly initialized.')
end

z_dot = pc2_filter_dot*wt + wt_dot*pc2_filter;

sys = [Ts_dot
        veh_speed_dot
        pc2_filter_dot
        wt_dot
        wcr_dot
        z_dot];

% ====================
% Compute State Resets
% ====================

% The reset is applied upon entering the discrete state.
if ismember(dynamic_mode,[FIRST,TORQUE12,TORQUE21])
    wcr_reset = R1*wt;
elseif ismember(dynamic_mode,[INERTIA12,INERTIA21])
    wcr_reset = wcr;
elseif ismember(dynamic_mode,SECOND)
    wcr_reset = R2*wt;
else
    error('hsprb_dynamics: dynamic_mode is not properly initialized.')
end

A = eye(6);
A(5,5) = 0;
B = zeros(6,1);
B(5) = wcr_reset;

reset.A = A;
reset.B = B;

return



















    
    
    
    
    







































                            
                            
                            
                            



