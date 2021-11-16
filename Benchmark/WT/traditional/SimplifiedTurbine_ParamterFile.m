%% SimplifiedTurbine_ParameterFile.m
function [Parameter] = SimplifiedTurbine_ParamterFile(Parameter)
% Contains all parameters for the 5MW NREL WT 
% (C) 2015 General Electic Global Research - all rights reserved

%% ========================================================================
% global constants %
Parameter.rho          = 1.225;                     % kg/m^3                    air density
Parameter.Rotor        = 126/2;                     % m                         rotor
Parameter.lambda_opt   = 7.55;                      % -                         tip speed ratio
Parameter.Omega_rated  = rpm2radPs(12.1);           % rpm                        
Parameter.wind_rated   = 11.2;
Parameter.eta          = .944;


Parameter.DT=0.01;

%% ========================================================================
% Turbine Dynamics
% Data taken from 
% D. Schlipf, D. Schlipf and M. Kuhn, 'Nonlinear model predictive control
% of wind turbines using LIDAR', Wind Energy, vol .13, pp. 1107-1129, 2013.
% and
% J. Jonkman,S. Butterfield, W. Musial, and G Scott, 'Definition of a 5-MW 
% reference wind turbine for offshore system development.” National 
% Renewable Energy Laboratory, Report NREL/TP-500-38060, 2009.


% fore-aft tower dynamics 
Parameter.h_H   = 90;                        % m                         Hub height
Parameter.m_T   = 522617;                    % kg                        Mass Tower + Monopile (Support Shallow)
Parameter.m_N   = 240000;                    % kg                        Mass Nacelle
Parameter.m_H   = 56780;                     % kg                        Mass Hub
Parameter.m_B   = 17740;                     % kg                        Mass Blade
Parameter.f_0   = 0.31994;                   % Hz                        naturall frequency 1st Tower FA
Parameter.d_s   = 0.01;                      % -                         Structural Damping ratio

Parameter.mTe   = 0.25*Parameter.m_T+Parameter.m_N+Parameter.m_H+3*Parameter.m_B;    % kg       tower equivalent modal mass ([Gasch] p.294)
Parameter.cTe   = 4*pi*Parameter.mTe*Parameter.d_s*Parameter.f_0;                   % kg/s     tower structual damping (sigma=C_T/(2M_T)=D*w_0, w_0=f_0*2*pi, [Gasch] p.294)
Parameter.kTe   = Parameter.mTe*(2*pi*Parameter.f_0)^2;                            % kg/s^2   tower bending stiffness (w_0=sqrt(K_T/M_T),[Gasch] p.294)

% Drive train dynamics
Parameter.J_H     = 115926;                  % kgm^2                     Hub Inertia About Shaft Axis
Parameter.J_B     = 11776047;                % kgm^2                     Second Mass Moment of Inertia (w.r.t. Root)
Parameter.J_G     = 534.116;                 % kgm^2                     Generator Inertia About High-Speed Shaft
Parameter.GBRatio = 1/97;                      % -                       Gearbox ratio

Parameter.Inertia = Parameter.J_H + 3*Parameter.J_B + Parameter.J_G*Parameter.GBRatio^2;       % kgm^2       sum of the moments of inertia about the rotation axis of the rotor hub and blades

Parameter.Omega_g_rated = Parameter.Omega_rated/Parameter.GBRatio;
%% ==========================================================================================
% VS Control (Paramters taken from DISCON.f90)
% fixed paramters
Parameter.GenEff                  = 0.944;   
Parameter.VSControl.VS_RtPwr      = 5296610.0;         % Rated generator generator power in Region 3, Watts. -- chosen to be 5MW divided by the electrical generator efficiency of 94.4%

Parameter.VSControl.VS_Rgn3MP     =       0.01745329;  % Minimum pitch angle at which the torque is computed as if we are in region 3 regardless of the generator speed, rad. -- chosen to be 1.0 degree above PC_MinPit
Parameter.VSControl.VS_CtInSp     =      70.16224;     % Transitional generator speed (HSS side) between regions 1 and 1 1/2, rad/s.
Parameter.VSControl.VS_MaxRat     =   15000.0;         % Maximum torque rate (in absolute value) in torque controller, N-m/s.
Parameter.VSControl.VS_MaxTq      =   47402.91;        % Maximum generator torque in Region 3 (HSS side), N-m. -- chosen to be 10% above VS_RtTq = 43.09355kNm
Parameter.VSControl.VS_Rgn2K      =       2.332287;    % Generator torque constant in Region 2 (HSS side), N-m/(rad/s)^2.
Parameter.VSControl.VS_Rgn2Sp     =      91.21091;     % Transitional generator speed (HSS side) between regions 1 1/2 and 2, rad/s.
Parameter.VSControl.VS_RtGnSp     =     121.6805;      % Rated generator speed (HSS side), rad/s. -- chosen to be 99% of PC_RefSpd
Parameter.VSControl.VS_RtPwr      = 5296610.0;         % Rated generator generator power in Region 3, Watts. -- chosen to be 5MW divided by the electrical generator efficiency of 94.4%
Parameter.VSControl.VS_SlPc       =      10.0;         % Rated generator slip percentage in Region 2 1/2, %.


% VS Control computed parameters
Parameter.VSControl.VS_SySp    = Parameter.VSControl.VS_RtGnSp/( 1.0 +  0.01*Parameter.VSControl.VS_SlPc );
Parameter.VSControl.VS_Slope15 = ( Parameter.VSControl.VS_Rgn2K*Parameter.VSControl.VS_Rgn2Sp*Parameter.VSControl.VS_Rgn2Sp )/( Parameter.VSControl.VS_Rgn2Sp - Parameter.VSControl.VS_CtInSp );
Parameter.VSControl.VS_Slope25 = ( Parameter.VSControl.VS_RtPwr/Parameter.VSControl.VS_RtGnSp           )/( Parameter.VSControl.VS_RtGnSp - Parameter.VSControl.VS_SySp   );

if ( Parameter.VSControl.VS_Rgn2K == 0.0 )    % .TRUE. if the Region 2 torque is flat, and thus, the denominator in the ELSE condition is zero
    Parameter.VSControl.VS_TrGnSp = Parameter.VSControl.VS_SySp;
else                      % .TRUE. if the Region 2 torque is quadratic with speed
    Parameter.VSControl.VS_TrGnSp = ( Parameter.VSControl.VS_Slope25 - sqrt( Parameter.VSControl.VS_Slope25*( Parameter.VSControl.VS_Slope25 - 4.0*Parameter.VSControl.VS_Rgn2K*Parameter.VSControl.VS_SySp ) ) )/( 2.0*Parameter.VSControl.VS_Rgn2K );
end

Parameter.VSControl.FilterConstant = 1/(0.25*2*pi); 


%% ================================================================
% Collective Blade Pitch Controller
 
% Pitch actuator system
% Data taken from 
% D. Schlipf, D. Schlipf and M. Kuhn, 'Nonlinear model predictive control
% of wind turbines using LIDAR', Wind Energy, vol .13, pp. 1107-1129, 2013.

Parameter.Pitch.omega  = 2*pi;        % rad/s          undamped natural frequency of the blade pitch actuator 
Parameter.Pitch.xi     = 0.7;      % -              damping factor of the blade pitch actuator   
Parameter.Pitch.Delay  = 0.3545;         % s              pitch delay (matlab risetime of second order lag)

Parameter.Pitch.FilterConstant = 1/(0.25*2*pi);


Parameter.Pitch.RateLimit   = deg2rad(8);       % [rad/s]
Parameter.Pitch.Max         = deg2rad(90);      % [rad]
Parameter.Pitch.Min = 0;                % [rad]

% Collective Pitch Controller (Paramters taken from DISCON.f90)
Parameter.Pitch.CPC.GS_factor  = 0.1099965;
Parameter.Pitch.CPC.KI         = 0.008068634; 
Parameter.Pitch.CPC.KP         = 0.01882681;
Parameter.Pitch.CPC.TI         = Parameter.Pitch.CPC.KP/Parameter.Pitch.CPC.KI;
Parameter.Pitch.CPC.AntiWindUp = 1;

%% ========================================================================
% Initial conditions for integrators
Parameter.Pitch.Theta0    = interp1(Parameter.InitialConditions.v0,Parameter.InitialConditions.pitch,Parameter.v0_0,'linear','extrap');
Parameter.Pitch.ThetaDot0 = 0;
% 
Parameter.xt_dot0          = 0;
Parameter.xT0              = (Parameter.v0_0^2*1/2*Parameter.rho*pi*Parameter.Rotor^2)/Parameter.kTe;

Parameter.Omega0  = interp1(Parameter.InitialConditions.v0,Parameter.InitialConditions.Omega,Parameter.v0_0,'linear','extrap');
Parameter.Omega_g0= Parameter.Omega0/Parameter.GBRatio;