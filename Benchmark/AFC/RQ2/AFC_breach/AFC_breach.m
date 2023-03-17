clear;
close all;
bdclose('all');
%% add path
if ismac() == 1
    % MacOS
    addpath(genpath('/Users/ldy/git/Benchmarks/tools/breach'));
    addpath(genpath('/Users/ldy/git/Benchmarks/AFC/'));
elseif isunix() == 1
    % Linux
    addpath(genpath('/home/ubuntu/git/Benchmarks/tools/breach'));
    addpath(genpath('/home/ubuntu/git/Benchmarks/AFC'));
elseif ispc() == 1
    % Windows
    % addpath(genpath(''));
end
%% set parameters
tool_name = 'breach';
% model name
mdl_name = 'AFC';
%%%%%%%%%%%%%%%%%%%% current model
mdl_list = {'AFC_hybrid_AI','AFC_hybrid_random_1','AFC_hybrid_random_01','AFC_hybrid_average','AFC_hybrid'};  % 'AFC_RL_1', 'AFC_RL_2', 'AFC_T', 'AFC_NN_3_10','AFC_NN_3_15','AFC_NN_3_20','AFC_NN_4_10','AFC_NN_4_15','AFC_NN_4_20','AFC_NN_5_10','AFC_NN_6_20'
%%%%%%%%%%%%%%%%%%%% agent list
agent_list = {'AFC_DDPG_Agent_8_28','AFC_DDPG_Agent_9_13','AFC_DDPG_Agent_9_18','AFC_A2C_Agent_9_18','AFC_PPO_Agent_9_18','AFC_SAC_Agent_9_18'};
hybrid_agent_list = {'AFC_DDPG_Agent_8_28'};
%%%%%%%%%%%%%%%%%%%%
% model parameters
global T Ts fuel_inj_tol MAF_sensor_tol AF_sensor_tol pump_tol kappa_tol tau_ww_tol fault_time kp ki mu_tol time_tol control_switch max_mu;
global min_Engine_Speed max_Engine_Speed min_Pedal_Angle max_Pedal_Angle;
global isTerminate switch_mu;

T = 30;
Ts = 0.1;
fuel_inj_tol=1.0;
MAF_sensor_tol=1.0;
AF_sensor_tol=1.0;
pump_tol=1;
kappa_tol=1;
tau_ww_tol=1;
fault_time=50;
kp=0.04;
ki=0.14;
%The engine speed is constrained to the range [900,1100].
min_Engine_Speed = 900;
max_Engine_Speed = 1100;
%The pedal angle is constrained to the range [8.8,61.1].
min_Pedal_Angle = 8.8;
max_Pedal_Angle = 61.1;
% RL parameters
% max_mu = 0.05;        % mu from STL
% mu_tol = 0.1;         % mu value to terminate the episode
% time_tol = 5;         % time tolerence, to avoid the initial large mu value
% hybrid control parameters
max_mu = 0.03;         % mu from STL
mu_tol = 0.1;          % mu value to terminate the episode
time_tol = 5;          % time tolerence, to avoid the initial large mu value
isTerminate = 0;        % never terminate sim in middle
control_switch = 0;     % 0: RL controller, 1: original controller
switch_mu = 0.12;       % hybrid: if mu > threshold -> switch to RL controller
% falsification parameters
trials = 30;
solver_list = {'cmaes','GNM'}; % default: global_nelder_mead
%%%%%%%%%%%%%%%%%%%%
phi1 = STL_Formula('phi1', 'alw_[0,30](AF[t] < 1.2*14.7 and AF[t] > 0.8*14.7)');
%%%%%%%%%%%%%%%%%%%% phi list
spec_list = {phi1};

InitBreach;
for solver_index = 1:numel(solver_list)
    cur_solver = solver_list{1, solver_index};
    for spec_index = 1:numel(spec_list)
        cur_spec = spec_list{1,spec_index};
        for mdl_index = 1:numel(mdl_list)
            cur_mdl = mdl_list{1,mdl_index};
            if strcmp(cur_mdl,'AFC_T') || contains(cur_mdl, 'AFC_NN')
                % falsification
                cur_spec = spec_list{1,spec_index};
                [obj_best,num_sim,time] = breach_fal(cur_mdl,cur_spec,trials,cur_solver);
                % save
                if strcmp(cur_mdl,'AFC_T')
                    file_name = [mdl_name,'_',num2str(spec_index),'_','T','_',tool_name,'_',cur_solver,'_result.mat'];
                else
                    file_name = [mdl_name,'_',num2str(spec_index),'_',cur_mdl,'_',tool_name,'_',cur_solver,'_result.mat'];
                end
                save(file_name, 'num_sim', 'time', 'obj_best');
                
            elseif strcmp(cur_mdl,'AFC_RL_1')
                for agent_index = 1:numel(agent_list)
                    % load agent
                    cur_agent = agent_list{1,agent_index};
                    
                    if contains(cur_agent,'DDPG')
                        agent = load([cur_agent,'.mat']);
                        agent = agent.agent;
                        % falsification
                        [obj_best,num_sim,time] = breach_fal(cur_mdl,cur_spec,trials,cur_solver);
                        % save
                        new_agent_name = replace(cur_agent, mdl_name, '');
                        file_name = [mdl_name,'_',num2str(spec_index),new_agent_name,'_',tool_name,'_',cur_solver,'_result.mat'];
                        save(file_name, 'num_sim', 'time', 'obj_best');
                    else
                        continue;
                    end
                end
            elseif strcmp(cur_mdl,'AFC_RL_2')
                for agent_index = 1:numel(agent_list)
                    % load agent
                    cur_agent = agent_list{1,agent_index};
                    if ~contains(cur_agent,'DDPG')
                        agent = load([cur_agent,'.mat']);
                        agent = agent.agent;
                        % falsification
                        [obj_best,num_sim,time] = breach_fal(cur_mdl,cur_spec,trials,cur_solver);
                        % save
                        new_agent_name = replace(cur_agent, mdl_name, '');
                        file_name = [mdl_name,'_',num2str(spec_index),new_agent_name,'_',tool_name,'_',cur_solver,'_result.mat'];
                        save(file_name, 'num_sim', 'time', 'obj_best');
                    else
                        continue;
                    end
                end
            elseif contains(cur_mdl, 'hybrid')  % for DDPG and NN
                for hybrid_agent_index = 1:numel(hybrid_agent_list)
                    % load agent for hybrid control
                    cur_hybrid_agent = hybrid_agent_list{1,hybrid_agent_index};
                    agent = load([cur_hybrid_agent,'.mat']);
                    agent = agent.agent;
                    % falsification
                    [obj_best,num_sim,time] = breach_fal(cur_mdl,cur_spec,trials,cur_solver);
                    % save
                    new_hybrid_agent_name = replace(cur_hybrid_agent, mdl_name, '');
                    file_name = [cur_mdl,'_',num2str(spec_index),new_hybrid_agent_name,'_',tool_name,'_',cur_solver,'_result.mat'];
                    save(file_name, 'num_sim', 'time', 'obj_best');
                end
            else
                disp("Check your model.");
            end
        end
    end
end


function [obj_best,num_sim,time] = breach_fal(mdl, phi, trials, solver_name)
global T Ts fuel_inj_tol MAF_sensor_tol AF_sensor_tol pump_tol kappa_tol tau_ww_tol fault_time kp ki mu_tol time_tol control_switch max_mu;
global min_Engine_Speed max_Engine_Speed min_Pedal_Angle max_Pedal_Angle;
global isTerminate switch_mu;

Br = BreachSimulinkSystem(mdl);
Br.Sys.tspan = 0:Ts:T;

cp_num = 5;
input_gen.type = 'UniStep';
input_gen.cp = cp_num;

Br.SetInputGen(input_gen);

for cpi = 0:input_gen.cp-1
    Pedal_Angle_sig = strcat('Pedal_Angle_u',num2str(cpi));
    Br.SetParamRanges({Pedal_Angle_sig},[min_Pedal_Angle max_Pedal_Angle]);
    
    Engine_Speed_sig = strcat('Engine_Speed_u', num2str(cpi));
    Br.SetParamRanges({Engine_Speed_sig},[min_Engine_Speed max_Engine_Speed]);
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

