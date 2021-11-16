%% SimplifiedTurbine_PostProcessing.m
% Computation of tower loads and rotor std
% (C) 2015 General Electic Global Research - all rights reserved

switch config.ProcessCase
    case ('SingleRun')
        IND                           = (xT_dot.time>Parameter.Time.cut_in) & (xT_dot.time<=Parameter.Time.cut_out);
        % time signals
        results.xT.time               = xT.time(IND);
        results.xT.signals.values     = xT.signals.values(IND);
        
        results.xT_dot.time           = xT_dot.time(IND);
        results.xT_dot.signals.values = xT_dot.signals.values(IND);
        
        results.Omega.time            = Omega.time(IND);
        results.Omega.signals.values  = Omega.signals.values(IND);
        
        results.Theta.time            = Theta.time(IND);
        results.Theta.signals.values  = Theta.signals.values(IND);
        
        results.Mg.time               = Mg.time(IND);
        results.Mg.signals.values     = Mg.signals.values(IND);
        
        % Calculate loads
        results.TwrDynLoads.time           = xT_dot.time(IND);
        results.TwrDynLoads.signals.values = Parameter.h_H*(Parameter.cTe*xT_dot.signals.values(IND) + Parameter.kTe*xT.signals.values(IND));
        results.TwrDEL                     = calc_DEL(xT_dot.time(IND),results.TwrDynLoads.signals.values);
        
        % Calculate STD (only calculate if URef > rated wind
        if Parameter.URef > Parameter.wind_rated
            results.Omega_std = std(results.Omega.signals.values);
        end
        
        % Calculate Power
        results.Power.time           = results.Mg.time;
        results.Power.signals.values = Parameter.eta.*results.Mg.signals.values.*results.Omega.signals.values*2*pi/60./Parameter.GBRatio;
 
    case ('AllCases') 
        IND   = (xT_dot.time>Parameter.Time.cut_in) & (xT_dot.time<=Parameter.Time.cut_out);
        % time signals
        results{i_ind,j_ind}.xT.time               = xT.time(IND);
        results{i_ind,j_ind}.xT.signals.values     = xT.signals.values(IND);
        
        results{i_ind,j_ind}.xT_dot.time           = xT_dot.time(IND);
        results{i_ind,j_ind}.xT_dot.signals.values = xT_dot.signals.values(IND);
        
        results{i_ind,j_ind}.Omega.time            = Omega.time(IND);
        results{i_ind,j_ind}.Omega.signals.values  = Omega.signals.values(IND);
        
        results{i_ind,j_ind}.Theta.time            = Theta.time(IND);
        results{i_ind,j_ind}.Theta.signals.values  = Theta.signals.values(IND);
        
        results{i_ind,j_ind}.Mg.time               = Mg.time(IND);
        results{i_ind,j_ind}.Mg.signals.values     = Mg.signals.values(IND);
        
        % Calculate loads
        results{i_ind,j_ind}.TwrDynLoads.time           = xT_dot.time(IND);
        results{i_ind,j_ind}.TwrDynLoads.signals.values = Parameter.h_H*(Parameter.cTe*xT_dot.signals.values(IND) + Parameter.kTe*xT.signals.values(IND));
       
        % Calculate Power
        results{i_ind,j_ind}.Power.time           = results{i_ind,j_ind}.Mg.time;
        results{i_ind,j_ind}.Power.signals.values = Parameter.eta.*results{i_ind,j_ind}.Mg.signals.values.*results{i_ind,j_ind}.Omega.signals.values*2*pi/60./Parameter.GBRatio;
        
end