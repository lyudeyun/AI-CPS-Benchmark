clear all;
close all;

addpath(genpath('/home/ubuntu/git/Benchmarks/tools/breach'));
addpath(genpath('/home/ubuntu/git/Benchmarks/SC'));

% addpath(genpath('/Users/ldy/git/Benchmarks/tools/breach'));
% addpath(genpath('/Users/ldy/git/Benchmarks/SC/'));

InitBreach;
tool_name = 'breach';
% model name
mdl_name = 'SC';
%%%%%%%%%%%%%%%%%%%% current model
mdl_list = {'SC_PID','SC_RNN','SC_RL'};
%%%%%%%%%%%%%%%%%%%% agent list
agent_list = {'SC_A2C_Agent_9_20','SC_DDPG_Agent_9_20','SC_PPO_Agent_9_20','SC_TD3_Agent_9_20'};  % 
%%%%%%%%%%%%%%%%%%%%
% model parameters
global T Ts min_Fs max_Fs change_freq P_step P_init P_final P_terminate time_tol seed;
T = 35;
Ts = 0.1;
min_Fs = 3.99;
max_Fs = 4.01;      % maximum steam flowrate
% RL parameters
change_freq = 5;    % input signal change value every change_freq sec
P_step = 2;         % ref pressure step time
P_init = 90;        % initial ref pressure
P_final = 87;       % final ref pressure
P_terminate = 100;    % error to terminate episode
time_tol = 3;
seed = 0;
% falsification parameters
trials = 30;
solver_list = {'GNM'}; % default: global_nelder_mead
%%%%%%%%%%%%%%%%%%%%
phi1 = STL_Formula('phi1','alw_[30 35] (Out4[t] >= 86.5 and Out4[t] <= 87.5)');
%%%%%%%%%%%%%%%%%%%% phi list
spec_list = {phi1};

for solver_index = 1:numel(solver_list)
    cur_solver = solver_list{1, solver_index};
    for mdl_index = 1:numel(mdl_list)
        cur_mdl = mdl_list{1,mdl_index};
        if strcmp(cur_mdl,'SC_PID')
            for spec_index = 1: numel(spec_list)
                % falsification
                [obj_best,num_sim,time] = breach_fal(cur_mdl,spec_list{1,spec_index},trials,cur_solver);
                % save
                file_name = [mdl_name,'_',num2str(spec_index),'_','T','_',tool_name,'_',cur_solver,'_Result.mat'];
                save(file_name, 'num_sim', 'time', 'obj_best');
            end
        elseif strcmp(cur_mdl,'SC_RL')
            for agent_index = 1:numel(agent_list)
                for spec_index = 1: numel(spec_list)
                    % load agent
                    agent_name = agent_list{1,agent_index};
                    agent = load([agent_name,'.mat']);
                    agent = agent.agent;
                    % falsification
                    [obj_best,num_sim,time] = breach_fal(cur_mdl,spec_list{1,spec_index},trials,cur_solver);
                    % save
                    new_agent_name = replace(agent_name, mdl_name, '');
                    file_name = [mdl_name,'_',num2str(spec_index),new_agent_name,'_',tool_name,'_',cur_solver,'_Result.mat'];
                    save(file_name, 'num_sim', 'time', 'obj_best');
                end
            end
        elseif strcmp(cur_mdl,'SC_RNN')
            for spec_index = 1: numel(spec_list)
                % falsification
                [obj_best,num_sim,time] = breach_fal(cur_mdl,spec_list{1,spec_index},trials,cur_solver);
                % save
                file_name = [mdl_name,'_',num2str(spec_index),'_','RNN','_',tool_name,'_',cur_solver,'_Result.mat'];
                save(file_name, 'num_sim', 'time', 'obj_best');
            end
        else
            disp("Check your model.");
        end
    end
end


function [obj_best,num_sim,time] = breach_fal(mdl, phi, trials, solver_name)
global T Ts min_Fs max_Fs change_freq P_step P_init P_final P_terminate time_tol seed;

Br = BreachSimulinkSystem(mdl);
Br.Sys.tspan = 0:Ts:T;

cp_num = 7;
input_gen.type = 'UniStep';
input_gen.cp = cp_num;

Br.SetInputGen(input_gen);

for cpi = 0:input_gen.cp-1
    in1_sig = strcat('Input_u', num2str(cpi));
    Br.SetParamRanges({in1_sig},[min_Fs max_Fs]);
end

falsified = [];
time = [];
obj_best = [];
num_sim = [];
for n = 1:trials
    falsif_pb = FalsificationProblem(Br,phi);
    falsif_pb.max_obj_eval = 300;
    if strcmp(solver_name, 'cmaes')
        falsif_pb.setup_solver(solver_name);
    elseif strcmp(solver_name, 'GNM')
    end
    falsif_pb.solve();
    if falsif_pb.obj_best < 0
        falsified = [falsified;1];
    else
        falsified = [falsified;0];
    end
    num_sim = [num_sim;falsif_pb.nb_obj_eval];
    time = [time;falsif_pb.time_spent];
    obj_best = [obj_best;falsif_pb.obj_best];
end
end    
