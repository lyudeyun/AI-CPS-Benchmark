%% SimplifiedTurbine_Config.m
% Configuration file for SimplifiedWTModel.slx
% (C) 2015 General Electic Global Research - all rights reserved

%% Add some path
config.wafo_path            = 'wafo/wafo/'; 
% this is a full path to your WAFO toolbox (version 2.5 is used), DO NOT include WAFO in your set
% paths in Matlab, it will probably interfere with other toolboxes
config.flname               = 'results.mat';
% this is were Matlab will save the results, the structure is similar to the
% ClassA.mat file, the rows corresponding to the different wind speeds, and
% the columns corresponding to the individual seeds. The 4th column
% contains the global postprossing for the three seeds (for ProcessCase =
% AllCases)
%% ========================================================================
config.ProcessCase          = 'SingleRun';  % Options 'SingleRun' or 'AllCases'
Parameter.URef              = 12;           % wind speed; only valid in combination with 'SingleRun'

if strcmp(config.ProcessCase,'AllCases')
    Parameter.URef = [4:2:24];
end

% ========================================================================
%% Pitch Dynamics
Parameter.Pitch.ActuatorType = 1;   %1==2nd order lag, 2==1st order lag, 3==time delay
