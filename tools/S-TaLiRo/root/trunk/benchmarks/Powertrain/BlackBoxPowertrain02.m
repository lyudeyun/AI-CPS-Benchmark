% This function also returns the derivative 

function [T XT YT LT CLG Guards] = BlackBoxPowertrain02(X0,TS, steptime, ~)

%% TODO %% 
% Use TS to change simulation time
%%

% Initialization of parameters for the Powertrain_CheckMate_ver Simulink
% model.

% The model is described in detail in the technical report:
% Alongkrit Chutinan and Kenneth R. Butts, Dynamic Analysis of Hybrid 
% System Models for Design Validation
%
% The current modifications so the model can run under CheckMate 3.6 were
% done by:
% Yashwanth Annapureddy and Georgios Fainekos, Arizona State University
if nargin < 3
    steptime = [];
end

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

m_s_to_km_h = 3.6;          % conversion factor between m/s and km/h; unitless
pc2_torque_phase = .4;      % ratio of Pc2max to be applied initially as pressure 
                            % offset
                            
% ==============================
% Model Inputs (Assumed Constant)
% ==============================
tps= X0(1);
assignin('base','tps',tps);
grade = X0(2);
assignin('base','grade',grade);

% values which are added out of necessity
Isi12 = It1 - It;
shift_speed12 = powertrain_shift_speed_12(tps);
shift_speed21 = powertrain_shift_speed_21(tps);

% ====================================
% Compute other intermediate variables
% ====================================

% Engine torque
Tt = 1.7*tps+30;

% veh_speed >= shift_speed12
C = [0 -1 0 0 0 0];
d = -shift_speed12;
shift_speed12 = linearcon([],[],C,d);
assignin('base','shift_speed12',shift_speed12);
C_ss12 = C;
d_ss12 = d;

% veh_speed <= shift_speed21
C = [0 1 0 0 0 0];
d = shift_speed21;
shift_speed21 = linearcon([],[],C,d);
assignin('base','shift_speed21',shift_speed21);
C_ss21 = C;
d_ss21 = d;

% (1-R1/R2)*wt >= 0
C = [0 0 0 -(1 - R1/R2) 0 0];
d = 0;
first_c2slip_pos = linearcon([],[],C,d);
assignin('base','first_c2slip_pos',first_c2slip_pos);
C_ss41a = C;
d_ss41a = d;

% (1-R1/R2)*wt >= -.5
C = [0 0 0 -(1 - R1/R2) 0 0];
d = 0.5;
first_c2slip_dz = linearcon([],[],C,d);
assignin('base','first_c2slip_dz',first_c2slip_dz);

% equation (22) > 1
C = [0 0 -c2_mu1*AR2 -c2_mu2*(1-R1/R2)*AR2*pc2_torque_phase*Pc2max 0 -c2_mu2*(1-R1/R2)*AR2];
d = c2_mu1*AR2*pc2_torque_phase*Pc2max - 1;
Tc2_gt1_1st_tg2_c2slip_pos = linearcon([],[],C,d);
assignin('base','Tc2_gt1_1st_tg2_c2slip_pos',Tc2_gt1_1st_tg2_c2slip_pos);
 
% equation (23) > 1
C = [0 0 -c2_mu1*AR2 c2_mu2*(1-R1/R2)*AR2*pc2_torque_phase*Pc2max 0 c2_mu2*(1-R1/R2)*AR2];
d = c2_mu1*AR2*pc2_torque_phase*Pc2max - 1;
Tc2_gt1_1st_tg2_c2slip_dz = linearcon([],[],C,d);
assignin('base','Tc2_gt1_1st_tg2_c2slip_dz',Tc2_gt1_1st_tg2_c2slip_dz);
 
% equation (24) > 1
C = [0 0 c2_mu1*AR2 -c2_mu2*(1-R1/R2)*AR2*pc2_torque_phase*Pc2max 0 -c2_mu2*(1-R1/R2)*AR2];
d = -c2_mu1*AR2*pc2_torque_phase*Pc2max - 1;
Tc2_gt1_1st_tg2_c2slip_neg = linearcon([],[],C,d);
assignin('base','Tc2_gt1_1st_tg2_c2slip_neg',Tc2_gt1_1st_tg2_c2slip_neg);
 
% equation (28) <= 0
C = [(1-Isi12/It1)*R1*Rd 0 -((Isi12/It1)*(1 - R1/R2) + R1/R2)*c2_mu1*AR2 -((Isi12/It1)*(1 - R1/R2) + R1/R2)*c2_mu2*(1 - R1/R2)*AR2*pc2_torque_phase*Pc2max 0 -((Isi12/It1)*(1 - R1/R2) + R1/R2)*c2_mu2*(1 - R1/R2)*AR2];
d = ((Isi12/It1)*(1 - R1/R2) + R1/R2)*c2_mu1*AR2*pc2_torque_phase*Pc2max - (Isi12/It1)*Tt;
RTsp1_lt0_tq12_tg2_c2slip_pos = linearcon([],[],C,d);
assignin('base','RTsp1_lt0_tq12_tg2_c2slip_pos',RTsp1_lt0_tq12_tg2_c2slip_pos);

% equation (29) <= 0
C = [(1-Isi12/It1)*R1*Rd 0 -((Isi12/It1)*(1 - R1/R2) + R1/R2)*c2_mu1*AR2 ((Isi12/It1)*(1 - R1/R2) + R1/R2)*c2_mu2*(1 - R1/R2)*AR2*pc2_torque_phase*Pc2max 0 ((Isi12/It1)*(1 - R1/R2) + R1/R2)*c2_mu2*(1 - R1/R2)*AR2];
d = ((Isi12/It1)*(1 - R1/R2) + R1/R2)*c2_mu1*AR2*pc2_torque_phase*Pc2max - (Isi12/It1)*Tt;
RTsp1_lt0_tq12_tg2_c2slip_dz = linearcon([],[],C,d);
assignin('base','RTsp1_lt0_tq12_tg2_c2slip_dz',RTsp1_lt0_tq12_tg2_c2slip_dz);
 
% equation (30) <= 0
C = [(1-Isi12/It1)*R1*Rd 0 ((Isi12/It1)*(1 - R1/R2) + R1/R2)*c2_mu1*AR2 -((Isi12/It1)*(1 - R1/R2) + R1/R2)*c2_mu2*(1 - R1/R2)*AR2*pc2_torque_phase*Pc2max 0 -((Isi12/It1)*(1 - R1/R2) + R1/R2)*c2_mu2*(1 - R1/R2)*AR2];
d = -((Isi12/It1)*(1 - R1/R2) + R1/R2)*c2_mu1*AR2*pc2_torque_phase*Pc2max - (Isi12/It1)*Tt;
RTsp1_lt0_tq12_tg2_c2slip_neg = linearcon([],[],C,d);
assignin('base','RTsp1_lt0_tq12_tg2_c2slip_neg',RTsp1_lt0_tq12_tg2_c2slip_neg);

% equation (32)
C = [0 0 0 1 -(1/R2) 0];
d = 0.5;
inert12_c2slip_neg = linearcon([],[],C,d);
assignin('base','inert12_c2slip_neg',inert12_c2slip_neg);
C_eq32 = C;
d_eq32 = d;

% equation (34)
C = [-(It/It2)*R2*Rd 0 -(c2_mu1*AR2) 0 0 0;
     (It/It2)*R2*Rd 0 -(c2_mu1*AR2) 0 0 0];
d = [(1 - It/It2)*Tt; -(1 - It/It2)*Tt];
abs_Tc2_gt_abs_RTc2updn_2nd_tg1 = linearcon([],[],C,d);
assignin('base','abs_Tc2_gt_abs_RTc2updn_2nd_tg1',abs_Tc2_gt_abs_RTc2updn_2nd_tg1);

% equation (36)
C = [0 0 0 -1 (1/R1) 0];
d = 0.5;
inert21_c1slip_dz = linearcon([],[],C,d);
assignin('base','inert21_c1slip_dz',inert21_c1slip_dz);

% equation (38) <= 1
C = [0 0 c2_mu1*AR2 0 0 c2_mu2*(1-R1/R2)*AR2];
d = 1;
Tc2_lt1_tq21_tg1_c2slip_pos = linearcon([],[],C,d);
assignin('base','Tc2_lt1_tq21_tg1_c2slip_pos',Tc2_lt1_tq21_tg1_c2slip_pos);
C_ss41a = [C_ss41a; C];
d_ss41a = [d_ss41a; d];

% equation (39) <= 1
C = [0 0 c2_mu1*AR2 0 0 -c2_mu2*(1-R1/R2)*AR2];
d = 1;
Tc2_lt1_tq21_tg1_c2slip_dz = linearcon([],[],C,d);
assignin('base','Tc2_lt1_tq21_tg1_c2slip_dz',Tc2_lt1_tq21_tg1_c2slip_dz);
C_ss39 = C;
d_ss39 = d;

% equation (40) <= 1
 C = [0 0 -c2_mu1*AR2 0 0 c2_mu2*(1-R1/R2)*AR2];
 d = 1;
 Tc2_lt1_tq21_tg1_c2slip_neg = linearcon([],[],C,d);
 assignin('base','Tc2_lt1_tq21_tg1_c2slip_neg',Tc2_lt1_tq21_tg1_c2slip_neg);
 C_ss40 = C;
 d_ss40 = d;
 
% Initial Continous Set 
hsprb_ICS=linearcon([],[], [], []);
assignin('base','hsprb_ICS',hsprb_ICS);

% Analysis Region
hsprb_AR=linearcon([],[], [], []);
assignin('base','hsprb_AR',hsprb_AR);

%%%
if ~isempty(steptime)
    [T] = sim('Powertrain_CheckMate_ver',steptime);
else
    [T] = sim('Powertrain_CheckMate_ver',[0 TS]);
end

XT = states.signals.values(:,1:6);
Tr_tmp = XT(:,1);
Tr_tmp = [Tr_tmp;Tr_tmp(end)];
Tr_tmp(1) = [];
Tr_tmp = abs(XT(:,1)-Tr_tmp);
XT = [XT Tr_tmp];
YT = [];
LT = states.signals.values(:,end);

% [newmap,CLG] = extract_graph('first_gear','Powertrain_CheckMate_ver');

CLG{1} = [2];
CLG{2} = [1 3];
CLG{3} = [4];
CLG{4} = [1 3];

% Guards(1,2).A = C_ss12;
% Guards(1,2).b = d_ss12;
% 
% Guards(2,1).A = C_ss21;
% Guards(2,1).b = d_ss21;
% 
% Guards(2,3).A = C_eq32;
% Guards(2,3).b = d_eq32;
% 
% Guards(3,4).A = C_ss21;
% Guards(3,4).b = d_ss21;
% 
% C_ss41b = [0 0 0 -(1 - R1/R2) 0 0; 0 0 0 (1 - R1/R2) 0 0; C_ss39];
% d_ss41b = [0.5; 0; d_ss39];
% 
% C_ss41c = [0 0 0 (1 - R1/R2) 0 0; C_ss40];
% d_ss41c = [-0.5; d_ss40];
% 
% Guards(4,1).A = {C_ss41a C_ss41b C_ss41c};
% Guards(4,1).b = {d_ss41a d_ss41b d_ss41c};
% 
% Guards(4,3).A = C_ss12;
% Guards(4,3).b = d_ss12;

Guards(1,2).A = [C_ss12 0];
Guards(1,2).b = d_ss12;

Guards(2,1).A = [C_ss21 0];
Guards(2,1).b = d_ss21;

Guards(2,3).A = [C_eq32 0];
Guards(2,3).b = d_eq32;

Guards(3,4).A = [C_ss21 0];
Guards(3,4).b = d_ss21;

C_ss41b = [0 0 0 -(1 - R1/R2) 0 0 0 ; 0 0 0 (1 - R1/R2) 0 0 0; C_ss39 0];
d_ss41b = [0.5; 0; d_ss39];

C_ss41c = [0 0 0 (1 - R1/R2) 0 0 0; C_ss40 0];
d_ss41c = [-0.5; d_ss40];

Guards(4,1).A = {[C_ss41a [0;0]] C_ss41b C_ss41c};
Guards(4,1).b = {d_ss41a d_ss41b d_ss41c};

Guards(4,3).A = [C_ss12 0];
Guards(4,3).b = d_ss12;



