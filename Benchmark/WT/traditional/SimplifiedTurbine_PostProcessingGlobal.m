%% SimplifiedTurbine_PostProcessingGlobal.m
% Computation of tower loads and rotor std
% (C) 2015 General Electic Global Research - all rights reserved

%Global postprocessing after full run
    for i = 1:size(v0_cell,1)
        tmp_TwrDynLoads = [];
        tmp_Omega       = [];
        tmp_Power       = [];
        for j = 1:size(v0_cell,2)
            tmp_TwrDynLoads = [tmp_TwrDynLoads,results{i,j}.TwrDynLoads.signals.values'];
            tmp_Omega       = [tmp_Omega,results{i,j}.Omega.signals.values'];
            tmp_Power       = [tmp_Power,results{i,j}.Power.signals.values'];
        end
        time = [0:Parameter.DT:length(tmp_TwrDynLoads)*Parameter.DT-Parameter.DT];
        results{i,j+1}.DELTwrLoads = calc_DEL(time,tmp_TwrDynLoads);
        % Calculate STD (only calculate if URef > rated wind
        if URefVector(i) > Parameter.wind_rated
            results{i,j+1}.Omega_std   = std(tmp_Omega);
        else
            results{i,j+1}.Omega_std(i)   = NaN;
        end
        
        results{i,j+1}.Power_mean = mean(tmp_Power);
    end