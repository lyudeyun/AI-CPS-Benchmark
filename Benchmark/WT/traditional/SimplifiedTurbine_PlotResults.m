%% SimplifiedTurbine_PlotResults.m
% Plot results
% (C) 2015 General Electic Global Research - all rights reserved
close all
load(config.flname)

if strcmp(config.ProcessCase,'SingleRun')
    
    figure
    subplot(6,1,1)
    plot(v0_cell{iBin,iRandSeed}.time(121:2521), v0_cell{iBin,iRandSeed}.signals.values(121:2521),'LineWidth',2)
    title(['Results for single run with mean wind speed v0 = ', num2str(Parameter.URef),'m/s'])
    xlabel('time [s]')
    ylabel('wind speed [m/s]')
    xlim([30,630])
    
    subplot(6,1,2)
    plot(results.Power.time(:), results.Power.signals.values(:)*1e-6,'LineWidth',2)
    xlabel('time [s]')
    ylabel('Power [MW]')
    xlim([30,630])
    
    subplot(6,1,3)
    plot(results.Omega.time(:), results.Omega.signals.values(:),'LineWidth',2)
    xlabel('time [s]')
    ylabel('\Omega [rpm]')
    xlim([30,630])
    
    subplot(6,1,4)
    plot(results.Theta.time(:), results.Theta.signals.values(:),'LineWidth',2)
    ylabel('\theta [°]')
    xlabel('time [s]')
    xlim([30,630])
    
    subplot(6,1,5)
    plot(results.Mg.time(:), results.Mg.signals.values(:),'LineWidth',2)
    ylabel('M_g [Nm]')
    xlabel('time [s]')
    xlim([30,630])
    
    subplot(6,1,6)
    plot(results.TwrDynLoads.time(:), results.TwrDynLoads.signals.values(:),'LineWidth',2)
    ylabel('M_{yT} [Nm]')
    xlabel('time [s]')
    xlim([30,630])
end

if strcmp(config.ProcessCase,'AllCases')
    set(0,'DefaultFigureWindowStyle','docked') 
        
    for i = 1:length(Parameter.URef)
        DEL(i) = results{i,4}.DynTwrLoads;
    end
    
    figure;
    h  = bar(v_vec,DEL, 'FaceColor', [0    0.4470    0.7410], 'EdgeColor', [0    0.4470    0.7410]);
    xlim([3,25])
    xlabel('wind speed [m/s]')
    ylabel('Tower DEL [Nm]')
    title('Damage equivalent loads per wind speed')
    
end

