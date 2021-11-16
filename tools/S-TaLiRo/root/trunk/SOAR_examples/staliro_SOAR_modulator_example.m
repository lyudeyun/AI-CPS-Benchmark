% This is the set of benchmark problems in the report:
% "Probabilistic Temporal Logic Falsification of Cyber-Physical Systems" 
% H. Abbas, G. Fainekos, S. Sankaranarayanan, F. Ivancic, and A. Gupta,

% (C) Georgios Fainekos 2011 - Arizona State Univeristy

clear
model='modulator_3rd_order';
init_cond = [-.1 .1;-.1 .1;-.1 .1];
input_range{1} = [-.45 .45];
input_range{2} = [-.4 .4];
input_range{3} = [-.35 .35];
cp_array = [10];
phi = '[]a';

preds.str='a';
preds.A = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
preds.b = [1 1 1 1 1 1]';


disp(' ')
disp('   1 . [-.45 .45]')
disp('   2 . [-.4 .4]')
disp('   3 . [-.35 .35]')
disp(' ')
form_id = input('Choose the input range for the system:');

time = 9;

%% Set up and run SOAR optimizers

opt = staliro_options();
opt.optimization_solver = 'SOAR_Taliro_LocalGPs';
opt.runs = 10;                  % Set algorithmic macro-replications
opt.optim_params.n_tests = 100; % Set the length of each marco-replicaitons
opt.spec_space = 'X';
opt.seed = 12222;               % Set Optimization seed for repeatability
opt.optim_params.crowded_EI_flag = 0;       % Default value is 1,such that croweded EI is used 
opt.optim_params.crowding_threshold = 0.10; % Default value is 0.05
opt.optim_params.TR_lowpass_thresh = 0.05;  % Default value is 0.25
opt.optim_params.TR_highpass_thresh = 0.50; % Default value is 0.75
opt.optim_params.TR_delta = 0.7;            % Default value is 0.75
opt.optim_params.TR_gamma = 1.3;            % Default value is 1.25

[results_SOAR_LocalGPs, history_SOAR_LocalGPs] = staliro(model,model.init.cube,input_range,cp_array,phi{form_id},Pred,time,opt);

disp(' ') 
disp('**** Results for SOAR_Local_GP *****')
short_results_display(results_SOAR_LocalGPs,history_SOAR_LocalGPs);
disp('************************************')
disp(' ') 
input('Press any button to continue to next optimizer')

% Switch to SOAR with finite differencing and run 
opt.optimization_solver = 'SOAR_Taliro_FiniteDiff';

[results_SOAR_FiniteDiff, history_SOAR_FiniteDiff] = staliro(model,model.init.cube,input_range,cp_array,phi{form_id},Pred,time,opt);

disp(' ') 
disp('**** Results for SOAR_FiniteDiff *****')
short_results_display(results_SOAR_FiniteDiff,history_SOAR_FiniteDiff);
disp('************************************')
disp(' ') 
input('Press any button to continue to next optimizer')

% Switch to SOAR with SPSA and run 
opt.optimization_solver = 'SOAR_Taliro_SPSA';

[results_SOAR_SPSA, history_SOAR_SPSA] = staliro(model,model.init.cube,input_range,cp_array,phi{form_id},Pred,time,opt);

disp(' ') 
disp('**** Results for SOAR_SPSA *****')
short_results_display(results_SOAR_SPSA,history_SOAR_SPSA);
disp('************************************')
disp(' ') 
input('Press any button to continue to next optimizer')

%Switch to SOAR with 2SPSA and run 
opt.optimization_solver = 'SOAR_Taliro_2SPSA';

[results_SOAR_2SPSA, history_SOAR_2SPSA] = staliro(model,model.init.cube,input_range,cp_array,phi{form_id},Pred,time,opt);

disp(' ') 
disp('**** Results for SOAR_2SPSA *****')
short_results_display(results_SOAR_2SPSA,history_SOAR_2SPSA);
disp('************************************')
disp(' ') 

%% Function to quickly present results from the results/history structure returned by S-TaLiRo
function short_results_display(results,history)
    nT_SOAR=[]; %store number of simulations of falsifying runs
    nR_SOAR=[]; %store lowest robustness of non-falsifying runs
    numOfFals_SOAR=0; 
    Fals_Inps_SOAR=[]; %store the falsifying input of each falsifying run
    outer_runs_SOAR = length(history);

    runtimes_SOAR=[];
    for iii=1:outer_runs_SOAR
        if results.run(iii).falsified==1
            Fals_Inps_SOAR = [Fals_Inps_SOAR; results.run(iii).bestSample'];
            nT_SOAR=[nT_SOAR, results.run(iii).nTests];
            numOfFals_SOAR=numOfFals_SOAR+1;
        else
            nR_SOAR=[nR_SOAR; results.run(iii).bestRob];
        end
        runtimes_SOAR=[runtimes_SOAR;results.run(iii).time];
    end

    disp(['number of falsifications: ',num2str(numOfFals_SOAR),'/', num2str(outer_runs_SOAR)])
    disp(['Average number of runs to falsify: ', num2str(mean(nT_SOAR))])
    disp(['Median number of runs to falsify: ', num2str(median(nT_SOAR))])
end
