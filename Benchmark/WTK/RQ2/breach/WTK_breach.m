clear;
close all;
bdclose('all');
%% add path
if ismac() == 1
    % MacOS
    addpath(genpath('/Users/ldy/git/Benchmarks/tools/breach'));
    addpath(genpath('/Users/ldy/git/Benchmarks/WTK/'));
elseif isunix() == 1
    % Linux
    addpath(genpath('/home/ubuntu/git/Benchmarks/tools/breach'));
    addpath(genpath('/home/ubuntu/git/Benchmarks/WTK'));
elseif ispc() == 1
    % Windows
    % addpath(genpath(''));
end
%% set parameters
tool_name = 'breach';
% model name
mdl_name = 'WTK';
%%%%%%%%%%%%%%%%%%%% current model
mdl_list = {'WTK_RL_hybrid_random_1','WTK_RL_hybrid','WTK_RL_hybrid_random_01','WTK_RL_hybrid_average'}; % 'WTK_RL','WTK_PID'
%%%%%%%%%%%%%%%%%%%% agent list
agent_list = {'WTK_DDPG_Agent', 'WTK_TD3_Agent'};
hybrid_agent_list_1 = {'WTK_DDPG_Agent_9_15'};
hybrid_agent_list_2 = {'WTK_TD3_Agent_9_20'};
%%%%%%%%%%%%%%%%%%%%
% model parameters
global T Ts h_ref_min h_ref_max;
T = 24;
Ts = 0.1;
h_ref_min = 0;
h_ref_max = 20;
% RL parameters
desire_WL = 10;
% hybrid control parameters
isTerminate = 0;       % never terminate sim in middle
switch_error = 0.1;
% falsification parameters
trials = 30;
solver_list = {'cmaes','GNM'}; % default: global_nelder_mead
%%%%%%%%%%%%%%%%%%%%
phi1 = STL_Formula('phi1','alw_[5,5.9](abs(h_error[t]) < 1) and alw_[11,11.9](abs(h_error[t]) < 1) and alw_[17,17.9](abs(h_error[t]) < 1) and alw_[23,23.9](abs(h_error[t]) < 1)');
%%%%%%%%%%%%%%%%%%%% phi list
spec_list = {phi1};

InitBreach;
for solver_index = 1:numel(solver_list)
    cur_solver = solver_list{1, solver_index};
    for spec_index = 1:numel(spec_list)
        cur_spec = spec_list{1,spec_index};
        for mdl_index = 1:numel(mdl_list)
            cur_mdl = mdl_list{1,mdl_index};
            if strcmp(cur_mdl,'WTK_PID')
                % falsification
                cur_spec = spec_list{1,spec_index};
                [obj_best,num_sim,time] = breach_fal(cur_mdl,cur_spec,trials,cur_solver);
                % save
                file_name = [mdl_name,'_',num2str(spec_index),'_','T','_',tool_name,'_',cur_solver,'_result.mat'];
                save(file_name, 'num_sim', 'time', 'obj_best');
                
            elseif strcmp(cur_mdl,'WTK_RL')
                for agent_index = 1:numel(agent_list)
                    % load agent
                    cur_agent = agent_list{1,agent_index};
                    agent = load([cur_agent,'.mat']);
                    agent = agent.agent;
                    % falsification
                    [obj_best,num_sim,time] = breach_fal(cur_mdl,cur_spec,trials,cur_solver);
                    % save
                    new_agent_name = replace(cur_agent, mdl_name, '');
                    file_name = [mdl_name,'_',num2str(spec_index),new_agent_name,'_',tool_name,'_',cur_solver,'_result.mat'];
                    save(file_name, 'num_sim', 'time', 'obj_best');
                end
            elseif contains(cur_mdl, 'hybrid')
                for hybrid_agent_1_index = 1:numel(hybrid_agent_list_1)
                    % load agent_1 for hybrid control
                    cur_hybrid_agent_1 = hybrid_agent_list_1{1,hybrid_agent_1_index};
                    agent = load([cur_hybrid_agent_1,'.mat']);
                    agent = agent.agent;
                    for hybrid_agent_2_index = 1:numel(hybrid_agent_list_2)
                        % load agent_2 for hybrid control
                        cur_hybrid_agent_2 = hybrid_agent_list_2{1,hybrid_agent_2_index};
                        agent_2 = load([cur_hybrid_agent_2,'.mat']);
                        agent_2 = agent_2.agent;
                        % falsification
                        [obj_best,num_sim,time] = breach_fal(cur_mdl,cur_spec,trials,cur_solver);
                        % save
                        new_hybrid_agent_1_name = replace(cur_hybrid_agent_1, mdl_name, '');
                        new_hybrid_agent_2_name = replace(cur_hybrid_agent_2, mdl_name, '');
                        file_name = [cur_mdl,'_',num2str(spec_index),new_hybrid_agent_1_name,new_hybrid_agent_2_name,'_',tool_name,'_',cur_solver,'_result.mat'];
                        save(file_name, 'num_sim', 'time', 'obj_best');
                    end
                end
            else
                disp("Check your model.");
            end
        end
    end
end

function [obj_best,num_sim,time] = breach_fal(mdl, phi, trials, solver_name)
global T Ts h_ref_min h_ref_max;

Br = BreachSimulinkSystem(mdl);
Br.Sys.tspan = 0:Ts:T;

cp_num = 4;
input_gen.type = 'UniStep';
input_gen.cp = cp_num;

Br.SetInputGen(input_gen);

for cpi = 0:input_gen.cp-1
    h_ref_sig = strcat('h_ref_u', num2str(cpi));
    Br.SetParamRanges({h_ref_sig},[h_ref_min h_ref_max]);
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