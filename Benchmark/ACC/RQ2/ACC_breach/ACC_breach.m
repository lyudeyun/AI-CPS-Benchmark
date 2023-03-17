clear;
close all;
bdclose('all');
%% add path
if ismac() == 1
    % MacOS
    addpath(genpath('/Users/ldy/git/Benchmarks/tools/breach'));
    addpath(genpath('/Users/ldy/git/Benchmarks/ACC/'));
elseif isunix() == 1
    % Linux
    addpath(genpath('/home/ubuntu/git/Benchmarks/tools/breach'));
    addpath(genpath('/home/ubuntu/git/Benchmarks/ACC'));
elseif ispc() == 1
    % Windows
    % addpath(genpath(''));
end
%% set parameters
tool_name = 'breach';
% model name
mdl_name = 'ACC';
%%%%%%%%%%%%%%%%%%%% current model
mdl_list = {'ACC_RL_hybrid_random'};    % 'ACC_RL', 'ACC_T'  'ACC_RL_hybrid','ACC_RL_hybrid_average'
%%%%%%%%%%%%%%%%%%%% agent list
agent_list = {'ACC_DDPG_Agent_8_29','ACC_DDPG_Agent_9_11','ACC_DDPG_Agent_9_13','ACC_DDPG_Agent_9_22','ACC_SAC_Agent_9_11','ACC_TD3_Agent_9_11'};
hybrid_agent_list = {'ACC_DDPG_Agent_9_22'};
%%%%%%%%%%%%%%%%%%%%
% model parameters
global T Ts G_ego t_gap D_default v_set vmax_ego vmin_ego amin_ego amax_ego x0_lead v0_lead x0_ego v0_ego amin_lead amax_lead use_MPC;
global T_random isTerminate switch_distance;

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
% MPC_a_ego = 0;
% hybrid control parameters
T_random = 1;
isTerminate = 0;       % never terminate sim in middle
switch_distance = 5;
% falsification parameters
trials = 30;
solver_list = {'cmaes','GNM'}; % default: global_nelder_mead
%%%%%%%%%%%%%%%%%%%%
phi1 = STL_Formula('phi1', 'alw_[0,50](d_rel[t] - 1.4 * v_ego[t] >= 10)');
%%%%%%%%%%%%%%%%%%%% phi list
spec_list = {phi1};

InitBreach;
for solver_index = 1:numel(solver_list)
    cur_solver = solver_list{1, solver_index};
    for spec_index = 1:numel(spec_list)
        cur_spec = spec_list{1,spec_index};
        for mdl_index = 1:numel(mdl_list)
            cur_mdl = mdl_list{1,mdl_index};
            if strcmp(cur_mdl,'ACC_T')
                % falsification
                cur_spec = spec_list{1,spec_index};
                [obj_best,num_sim,time] = breach_fal(cur_mdl,cur_spec,trials,cur_solver);
                % save
                file_name = [mdl_name,'_',num2str(spec_index),'_','T','_',tool_name,'_',cur_solver,'_result.mat'];
                save(file_name, 'num_sim', 'time', 'obj_best');
                
            elseif strcmp(cur_mdl,'ACC_RL')
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
global T Ts amin_lead amax_lead;
 
Br = BreachSimulinkSystem(mdl);
Br.Sys.tspan = 0:Ts:T;

cp_num = 10;
input_gen.type = 'UniStep';
input_gen.cp = cp_num;

Br.SetInputGen(input_gen);

for cpi = 0:input_gen.cp-1
    in_lead_sig = strcat('in_lead_u', num2str(cpi));
    Br.SetParamRanges({in_lead_sig},[amin_lead amax_lead]);
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
