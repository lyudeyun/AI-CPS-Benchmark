%% SimplifiedTurbine_ParameterFile.m
% Main file to run SimplifiedWTModel.slx 
% (C) 2015 General Electic Global Research - all rights reserved

%% ========================================================================
clear all
close all
%clc

SimplifiedTurbine_Config;

% add some paths
addpath('tools/')
addpath('wind/')
addpath(config.wafo_path)

%load wind files
load('ClassA.mat')
load('ClassA_config.mat')

load('aeromaps3.mat');
Parameter.InitialConditions = load('InitialConditions');
% remove all unnecessary fields (otherwise Simulink will throw an error)
cT_modelrm = rmfield(cT_model,{'VarNames'});%,'RMSE','ParameterVar','ParameterStd','R2','AdjustedR2'});
cP_modelrm = rmfield(cP_model,{'VarNames'});%,'RMSE','ParameterVar','ParameterStd','R2','AdjustedR2'});

% initialize WAFO
initwafo 


%% single run or all wind cases

iBin = find(URefVector==Parameter.URef);
iRandSeed = 1;
switch config.ProcessCase
    case ('SingleRun')
        config.iBin                         = iBin;
        config.iRandSeed                    = iRandSeed;
        Parameter.v0                        = v0_cell{iBin,iRandSeed};
        Parameter.v0.signals.values         = Parameter.v0.signals.values';
        Parameter.TMax                      = v0_cell{iBin,iRandSeed}.time(end);
        config.WindFieldName                = FileNames{iBin,iRandSeed};
        % Time
        Parameter.Time.TMax                 = 630;              % [s]       duration of simulation
        Parameter.Time.dt                   = 0.01;           % [s]       time step of simulation
        Parameter.Time.cut_in               = 30;
        Parameter.Time.cut_out              = Parameter.Time.TMax;
        Parameter.v0_0 = Parameter.v0.signals.values(1);

        Parameter = SimplifiedTurbine_ParamterFile(Parameter);
        sim('SimplifiedWTModel.slx')
        SimplifiedTurbine_PostProcessing;
    case ('AllCases')
        %% Time
                Parameter.Time.TMax                 = 630;            % [s]       duration of simulation
                Parameter.Time.dt                   = 0.01;           % [s]       time step of simulation
                Parameter.Time.cut_in               = 30;
                Parameter.Time.cut_out              = Parameter.Time.TMax;
        for i_ind = 1:size(v0_cell,1)
            for j_ind = 1:size(v0_cell,2)
                clear Theta Theta_d xT xT_dot Omega lambda Mg
                iBin                                = i_ind;
                iRandSeed                           = j_ind;
                Parameter.v0                        = v0_cell{iBin,iRandSeed};
                Parameter.v0.signals.values         = Parameter.v0.signals.values';
                Parameter.TMax                      = v0_cell{iBin,iRandSeed}.time(end);
                config.WindFieldName                = FileNames{iBin,iRandSeed};
   
                Parameter.v0_0 = Parameter.v0.signals.values(1);
                
                Parameter = SimplifiedTurbine_ParamterFile(Parameter);
                sim('SimplifiedWTModel.slx')
                SimplifiedTurbine_PostProcessing;
            end
        end
        SimplifiedTurbine_PostProcessingGlobal
        
end

save(config.flname, 'results', 'Parameter', 'config')
SimplifiedTurbine_PlotResults

