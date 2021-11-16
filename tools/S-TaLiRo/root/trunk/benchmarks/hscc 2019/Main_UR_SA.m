% This file creates the results of experimental results section.
clear;
close;

% Simulation time & Requirements (run " help staliro" for more information)
global simin
SP.t0 = 0;
SP.tf = 35;
SP.pred(1).str = 'p1';  
SP.pred(1).A =  1;
SP.pred(1).b =  87.5;  
SP.pred(2).str = 'p2';  
SP.pred(2).A =  -1;
SP.pred(2).b =  -87;
SP.phi = '[]_[30,35] (p1/\p2)';  
model = @BlackBoxNN;        % Black box model that calls Simulink through staliro
nbstarts = 50;              % # of Runs
nbiter = 600;               % Max # of simulations
cp_array = 18;              % Change to 12 for HSCC results
input_range = [3.99 ,4.01]; % Input bounds


% UR ---------------------------------------
staliro_opt = staliro_options(); %(See staliro_options help file)
staliro_opt.interpolationtype = {'pconst'};
staliro_opt.optimization_solver = 'UR_Taliro';
staliro_opt.varying_cp_times = 1;
staliro_opt.runs = nbstarts;
staliro_opt.optim_params.n_tests = nbiter;
staliro_opt.black_box = 1;
staliro_opt.seed = 1;

[results4, history4] = staliro(model,[],input_range,cp_array,SP.phi,SP.pred,SP.tf,staliro_opt);
save('UR_steam', 'results4','staliro_opt')
disp(['# of falsifications: ',num2str(sum([results4.run.falsified].')),'/50'])
disp(['Avg min robustness: ',num2str(mean([results4.run.bestCost].'))])
disp(['Avg Execution time: ', num2str(mean([results4.run.time].'))])
disp(['Avg # of simulations: ', num2str(mean([results4.run.nTests].'))])


% SA ---------------------------------------
staliro_opt = staliro_options();
staliro_opt.interpolationtype = {'pconst'};
staliro_opt.varying_cp_times = 1;
staliro_opt.runs = nbstarts;
staliro_opt.black_box = 1;
staliro_opt.seed = 1;
staliro_opt.optim_params.n_tests = nbiter;

[results3, history3] = staliro(model,[],input_range,cp_array,SP.phi,SP.pred,SP.tf,staliro_opt);
save('SA_steam', 'results3','staliro_opt')
disp('For SA method:') 
disp(['# of falsifications: ',num2str(sum([results3.run.falsified].')),'/50'])
disp(['Avg min robustness: ',num2str(mean([results3.run.bestCost].'))])
disp(['Avg Execution time: ', num2str(mean([results3.run.time].'))])
disp(['Avg # of simulations: ', num2str(mean([results3.run.nTests].'))])



