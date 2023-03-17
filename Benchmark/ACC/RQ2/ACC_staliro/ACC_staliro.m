clear;
close all;
bdclose('all');
%% add path
if ismac() == 1
    % MacOS
    addpath(genpath('/Users/ldy/git/Benchmarks/tools/Copy_of_S-TaLiRo_new/root/'));
    addpath(genpath('/Users/ldy/git/Benchmarks/ACC/'));
elseif isunix() == 1
    % Linux
    addpath(genpath('/home/ubuntu/git/Benchmarks/tools/Copy_of_S-TaLiRo_new/root/'));
    addpath(genpath('/home/ubuntu/git/Benchmarks/ACC'));
elseif ispc() == 1
    % Windows
    % addpath(genpath(''));
end
%% set parameters
global cur_mdl agent G_ego t_gap D_default v_set vmax_ego vmin_ego amin_ego amax_ego x0_lead v0_lead x0_ego v0_ego amin_lead amax_lead use_MPC MPC_a_ego;
global opt Ts trials max_obj_eval;
global T_random isTerminate switch_distance;

mdl_name = 'ACC';
tool_name = 'staliro';
% model list
% mdl_list = {'ACC_RL_hybrid_random_01_staliro','ACC_RL_hybrid_random_1_staliro','ACC_RL_hybrid_staliro','ACC_RL_hybrid_average_staliro'};
mdl_list = {'ACC_RL_staliro', 'ACC_RL_hybrid_staliro', 'ACC_RL_hybrid_average_staliro'};
% agent list
agent_list = {'ACC_DDPG_Agent_9_22','ACC_SAC_Agent_9_11','ACC_TD3_Agent_9_11'}; % 'ACC_DDPG_Agent_8_29','ACC_DDPG_Agent_9_11','ACC_DDPG_Agent_9_13'
% hybrid agent list
hybrid_agent_list = {'ACC_DDPG_Agent_9_22'};
% solver list
solver_list = {'SA'};
% model parameters
T = 50;
Ts = 0.1;
% Specify the linear model for ego car.
G_ego = tf(1,[0.5,1,0]);
t_gap = 1.4;
D_default = 10;
% Specify the driver-set velocity in m/s.
v_set = 30;
% ego car minimum/maximum velocity
vmax_ego = 50;
vmin_ego = 0;
% the acceleration is constrained to the range [-3,2] (m/s^2).
amin_ego = -3;
amax_ego = 2;
% the velocity and the position of lead car
x0_lead = 70;
v0_lead = 40;
% the velocity and the position of ego car
x0_ego = 10;
v0_ego = 25;
% the acceleration is constrained to the range [-1,1] (m/s^2).
amin_lead = -1;
amax_lead = 1;
% RL parameters
use_MPC = 0;
MPC_a_ego = 0;
% hybrid control parameters
T_random = 1;
isTerminate = 0;       % never terminate sim in middle
switch_distance = 5;
% falsification parameters
cp_num = 10;
input_range = [amin_lead amax_lead];     % Input bounds, [amin amax; bmin bamx]
trials = 30;            % # of Runs
max_obj_eval = 300;     % Max # of simulations
%% Simulation time & Requirements (run " help staliro" for more information)
SP.t0 = 0;
SP.tf = T;
SP.pred(1).str = 'p1';
SP.pred(1).A = -1;
SP.pred(1).b = 0;
SP.phi = '[]_[0,50] p1'; % Make sure you specify the output mapping and its derivative if the requirement is not on the states
model = @BlackBoxACC;            % Black box model that calls Simulink through staliro
%%
rng('default')

for solver_index = 1:numel(solver_list)
    cur_solver = solver_list{1, solver_index};
    % select algorithm
    if strcmp(cur_solver,'SA')
        SA_config();
    elseif strcmp(cur_solver,'GA') 
        GA_config();
    elseif strcmp(cur_solver,'SOAR')
        SOAR_config();
    end
    for mdl_index = 1:numel(mdl_list)
        cur_mdl = mdl_list{1,mdl_index};
        if strcmp(cur_mdl,'ACC_T_staliro')
            % falsification
            [results, history] = staliro(model,[],input_range,cp_num,SP.phi,SP.pred,SP.tf,opt);
            % save
            file_name = [mdl_name,'_','T','_',tool_name,'_',cur_solver,'_result.mat'];
            save(file_name, 'results');
        elseif strcmp(cur_mdl,'ACC_RL_staliro')
            for agent_index = 1:numel(agent_list)
                % load agent
                agent_name = agent_list{1,agent_index};
                agent = load([agent_name,'.mat']);
                agent = agent.agent;
                % falsification
                % for GA
                % [results] = staliro(model,[],input_range,cp_num,SP.phi,SP.pred,SP.tf,opt);
                % for SA and others 
                [results, history] = staliro(model,[],input_range,cp_num,SP.phi,SP.pred,SP.tf,opt);
                % save
                new_agent_name = replace(agent_name, mdl_name, '');
                file_name = [mdl_name,new_agent_name,'_',tool_name,'_',cur_solver,'_result.mat'];
                save(file_name, 'results');
            end
        elseif contains(cur_mdl,'hybrid')
            for hybrid_agent_index = 1:numel(hybrid_agent_list)
                % load agent for hybrid control
                cur_hybrid_agent = hybrid_agent_list{1,hybrid_agent_index};
                agent = load([cur_hybrid_agent,'.mat']);
                agent = agent.agent;
                % falsification
                [results, history] = staliro(model,[],input_range,cp_num,SP.phi,SP.pred,SP.tf,opt);
                % save
                new_hybrid_agent_name = replace(cur_hybrid_agent, mdl_name, '');
                file_name = [cur_mdl,new_hybrid_agent_name,'_T_',tool_name,'_',cur_solver,'_result.mat'];
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

% try
%     [T, XT, YT] = feval(model,[],SP.tf,results.run.bestSample(1:end/2),results.run.bestSample(1+end/2:end));
%     figure
%     subplot(2,1,1)
%     plot(T, YT)
%     hold on
%     plot(30:35, SP.pred(1).b*ones(length(30:35)),'red')
%     plot(30:35, -SP.pred(2).b*ones(length(30:35)),'red')
%     title('Falsifying trajectory')
%     subplot(2,1,2)
%     plot(results.run.bestSample(1:end/2),results.run.bestSample(1+end/2:end))
%     title('Falsifying input')
% catch
%     TU = (0:opt.SampTime:SP.tf);
%     U = ComputeInputSignals(TU, results.run.bestSample, opt.interpolationtype, cp_num, input_range, SP.tf, opt.varying_cp_times)';
%     [T, XT, YT] = feval(model,[],SP.tf,TU,U);
%     figure
%     subplot(2,1,1)
%     plot(T, YT)
%     hold on
%     plot(30:35, SP.pred(1).b*ones(length(30:35)),'red')
%     plot(30:35, -SP.pred(2).b*ones(length(30:35)),'red')
%     title('Least robust trajectory found')
%     subplot(2,1,2)
%     plot(TU,U)
%     title('Best input found')
% end

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
%% GA config
function GA_config()
global opt Ts trials max_obj_eval;
opt = staliro_options();        %(See staliro_options help file)
opt.optimization_solver = 'GA_Taliro';
opt.black_box = 1;
opt.falsification = 1;
opt.SampTime = Ts;
opt.interpolationtype = {'pconst'};   % {'const', 'pconst'}
opt.runs = trials;
opt.optim_params.n_tests = max_obj_eval;
opt.seed = 1;
end
%% SOAR config
function SOAR_config()
global opt Ts trials max_obj_eval;
opt = staliro_options();
opt.optimization_solver = 'SOAR_Taliro_LocalGPs'; %'SOAR_Taliro_FiniteDiff'; %
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






