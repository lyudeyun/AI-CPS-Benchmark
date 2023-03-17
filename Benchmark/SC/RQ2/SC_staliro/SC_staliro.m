clear;
close all;
bdclose('all');
%% add path
if ismac() == 1
    % MacOS
    addpath(genpath('/Users/ldy/git/Benchmarks/tools/Copy_of_S-TaLiRo_new/root/'));
    addpath(genpath('/Users/ldy/git/Benchmarks/SC/'));
elseif isunix() == 1
    % Linux
    addpath(genpath('/home/ubuntu/git/Benchmarks/tools/Copy_of_S-TaLiRo_new/root/'));
    addpath(genpath('/home/ubuntu/git/Benchmarks/SC'));
elseif ispc() == 1
    % Windows
    % addpath(genpath(''));
end
%% set parameters
global cur_mdl agent  min_Fs max_Fs P_step P_init P_final P_terminate time_tol;
global opt Ts trials max_obj_eval;
mdl_name = 'SC';
tool_name = 'staliro';
% model list
mdl_list = {'SC_PID','SC_RL'};
% agent list
agent_list = {'SC_A2C_Agent_9_20','SC_DDPG_Agent_9_20','SC_PPO_Agent_9_20','SC_TD3_Agent_9_20'};
% solver list
solver_list = {'SA','SOAR'};
% model parameters
T = 35;
Ts = 0.1;
min_Fs = 3.99;
max_Fs = 4.01;      % maximum steam flowrate
% RL parameters
P_step = 2;         % ref pressure step time
P_init = 90;        % initial ref pressure
P_final = 87;       % final ref pressure
P_terminate = 100;    % error to terminate episode
time_tol = 3;
% falsification parameters
cp_num = 7;
input_range = [min_Fs max_Fs];     % Input bounds, [amin amax; bmin bamx]
trials = 30;            % # of Runs
max_obj_eval = 300;     % Max # of simulations
%% Simulation time & Requirements (run " help staliro" for more information)
SP.t0 = 0;
SP.tf = T;
SP.pred(1).str = 'p1';
SP.pred(1).A =  1;
SP.pred(1).b =  87.5;
SP.pred(2).str = 'p2';
SP.pred(2).A =  -1;
SP.pred(2).b =  -86.5;
SP.phi = '[]_[30,35] (p1/\p2)'; % Make sure you specify the output mapping and its derivative if the requirement is not on the states
model = @BlackBoxSC;            % Black box model that calls Simulink through staliro
%%
rng('default')

for solver_index = 1:numel(solver_list)
    cur_solver = solver_list{1, solver_index};
    % select algorithm
    if strcmp(cur_solver,'SA')
        SA_config();
    elseif strcmp(cur_solver,'SOAR')
        SOAR_config();
    end
    for mdl_index = 1:numel(mdl_list)
        cur_mdl = mdl_list{1,mdl_index};
        if strcmp(cur_mdl,'SC_PID')
            % falsification
            [results, history] = staliro(model,[],input_range,cp_num,SP.phi,SP.pred,SP.tf,opt);
            % save
            file_name = [mdl_name,'_','T','_',tool_name,'_',cur_solver,'_result.mat'];
            save(file_name, 'results');
        elseif strcmp(cur_mdl,'SC_RL')
            for agent_index = 1:numel(agent_list)
                % load agent
                agent_name = agent_list{1,agent_index};
                agent = load([agent_name,'.mat']);
                agent = agent.agent;
                % falsification
                [results, history] = staliro(model,[],input_range,cp_num,SP.phi,SP.pred,SP.tf,opt);
                % save
                new_agent_name = replace(agent_name, mdl_name, '');
                file_name = [mdl_name,new_agent_name,'_',tool_name,'_',cur_solver,'_result.mat'];
                save(file_name, 'results');
            end
        else
            disp("Check your model.");
        end
    end
end
%---------------------------------------------------------------------
% disp('Applied Pure SA method:')
% disp(['# of falsifications: ',num2str(sum([results.run.falsified].')),'/', num2str(max_obj_eval)])
% disp(['Avg min robustness: ',num2str(mean([results.run.bestCost].'))])
% disp(['Avg Execution time: ', num2str(mean([results.run.time].'))])
% disp(['Avg # of simulations: ', num2str(mean([results.run.nTests].'))])

try
    [T, XT, YT] = feval(model,[],SP.tf,results.run.bestSample(1:end/2),results.run.bestSample(1+end/2:end));
    figure
    subplot(2,1,1)
    plot(T, YT)
    hold on
    plot(30:35, SP.pred(1).b*ones(length(30:35)),'red')
    plot(30:35, -SP.pred(2).b*ones(length(30:35)),'red')
    title('Falsifying trajectory')
    subplot(2,1,2)
    plot(results.run.bestSample(1:end/2),results.run.bestSample(1+end/2:end))
    title('Falsifying input')
catch
    TU = (0:opt.SampTime:SP.tf);
    U = ComputeInputSignals(TU, results.run.bestSample, opt.interpolationtype, cp_num, input_range, SP.tf, opt.varying_cp_times)';
    [T, XT, YT] = feval(model,[],SP.tf,TU,U);
    figure
    subplot(2,1,1)
    plot(T, YT)
    hold on
    plot(30:35, SP.pred(1).b*ones(length(30:35)),'red')
    plot(30:35, -SP.pred(2).b*ones(length(30:35)),'red')
    title('Least robust trajectory found')
    subplot(2,1,2)
    plot(TU,U)
    title('Best input found')
end

%% SA config
function SA_config()
global opt Ts trials max_obj_eval;
opt = staliro_options();        %(See staliro_options help file)
opt.optimization_solver = 'SA_Taliro';
opt.black_box = 1;
opt.falsification = 1;
opt.SampTime = Ts;
opt.interpolationtype = {'pconst'};   % {'const', 'pconst'}
opt.runs = trials;
opt.optim_params.n_tests = max_obj_eval;
opt.seed = 1;
% opt.varying_cp_times = 1;
% opt.optim_params.dispStart= 0.9;  ??
% opt.optim_params.dispAdap = 2;
end
%% SOAR config
function SOAR_config()
global opt Ts trials max_obj_eval;
opt = staliro_options();
opt.optimization_solver = 'SOAR_Taliro_FiniteDiff'; % 'SOAR_Taliro_LocalGPs';
opt.black_box = 1;
opt.falsification = 1;
opt.SampTime = Ts;
opt.interpolationtype = {'pconst'};   % {'const', 'pconst'}
opt.runs = trials;                  % Set algorithmic macro-replications
opt.optim_params.n_tests = max_obj_eval; % Set the length of each marco-replicaitons
opt.seed = 1;               % Set Optimization seed for repeatability

% opt.taliro_metric = 'hybrid';   % Can alternative be set to 'hybrid_inf'
% opt.map2line = 1;               % CRITICAL - SOAR does not support hybrid output spaces currently
% opt.rob_scale = 1;              % 'a=1' in map2line ==> robustness of 40+ = 1;

% opt.optim_params.crowded_EI_flag = 0;       % Default value is 1,such that croweded EI is used
% opt.optim_params.crowding_threshold = 0.10; % Default value is 0.05
% opt.optim_params.TR_lowpass_thresh = 0.05;  % Default value is 0.25
% opt.optim_params.TR_highpass_thresh = 0.50; % Default value is 0.75
% opt.optim_params.TR_delta = 0.7;            % Default value is 0.75
% opt.optim_params.TR_gamma = 1.3;            % Default value is 1.25
end
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






